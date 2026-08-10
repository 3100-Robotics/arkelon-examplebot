package frc.robot.vision;

import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.constants.VisionAndPoseEstConstants;
import frc.robot.vision.MainVision.EstimateConsumer;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera implements Loggable {
  public final AprilTagFieldLayout tagLayout;
  public final Transform3d robotToCam;
  public final String camName;
  public final PhotonCamera camera;
  public final PhotonPoseEstimator photonEstimator;

  // public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 1);
  // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1.1, 1.1, 1.1);

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private Field2d simField;

  public SimCameraProperties simCamProps;
  public PhotonCameraSim sim;
  private Matrix<N3, N1> curStdDevs =
      VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  private final double distanceCutoffMeters;

  private String stdevMode = "";
  private String newStdDevMode = "";
  private String distMode = "";
  private String currentStrat = "";

  public boolean multiTagFound = false;
  public double latency = 0;
  private double avgDist = 0;
  private Pose3d intRawPose3d = Pose3d.kZero;

  private EstimateConsumer estimateConsumer =
      (Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) -> {};

  private Supplier<Pair<Rotation3d, Double>> headingSupplier = () -> Pair.of(Rotation3d.kZero, 0.0);

  public Camera(
      AprilTagFieldLayout atagfieldlayout,
      Transform3d robotCameraTransform,
      String camName,
      boolean doSim,
      SimCameraProperties simCamProps,
      boolean simDrawWireframe,
      boolean processedStream,
      boolean rawStream,
      double distanceCutoffMeters) {
    this.distanceCutoffMeters = distanceCutoffMeters;

    tagLayout = atagfieldlayout;
    this.robotToCam = robotCameraTransform;
    this.camName = camName;
    camera = new PhotonCamera(this.camName);
    photonEstimator = new PhotonPoseEstimator(this.tagLayout, this.robotToCam);

    if (doSim) {
      this.simCamProps = simCamProps;
      this.sim = new PhotonCameraSim(this.camera, this.simCamProps);
      this.sim.enableDrawWireframe(simDrawWireframe);
      this.sim.enableProcessedStream(processedStream);
      this.sim.enableRawStream(rawStream);
    }
  }

  public void setSimField(Field2d simField) {
    this.simField = simField;
  }

  public void setHeadingSupplier(Supplier<Pair<Rotation3d, Double>> headingSupplier) {
    this.headingSupplier = headingSupplier;
  }

  public void setPoseOutput(EstimateConsumer estimateConsumer) {
    this.estimateConsumer = estimateConsumer;
  }

  public void update() {
    var heading = headingSupplier.get();
    photonEstimator.addHeadingData(heading.getSecond(), heading.getFirst());

    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : camera.getAllUnreadResults()) {
      if (result.getTargets().size() == 1) {
        if (result.getTargets().get(0).poseAmbiguity > 0.2) {
          return;
        }
      }

      visionEst = photonEstimator.estimatePnpDistanceTrigSolvePose(result);
      // visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      currentStrat = "PnpDistanceTrigSolvePose";
      if (visionEst.isEmpty()) {
        currentStrat = "LowestAmbiguityPose";
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
      }

      result
          .getMultiTagResult()
          .ifPresentOrElse(
              e -> {
                multiTagFound = true;
              },
              () -> multiTagFound = false);

      updateEstimationStdDevs(visionEst, result.getTargets());

      latency = result.metadata.getLatencyMillis();

      if (Robot.isSimulation() && VisionAndPoseEstConstants.simulateCoproc) {
        visionEst.ifPresentOrElse(
            est ->
                simField
                    .getObject("VisionEstimation" + camName)
                    .setPose(est.estimatedPose.toPose2d()),
            () -> {
              simField.getObject("VisionEstimation" + camName).setPoses();
            });
      }

      visionEst.ifPresent(
          est -> {
            // Change our trust in the measurement based on the tags we can see
            var estStdDevs = getEstimationStdDevs();

            // if (est.estimatedPose.)
            // return;

            intRawPose3d = est.estimatedPose;
            estimateConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            // SmartDashboard.put("poseRaw_" + camName, est.estimatedPose.toPose2d());
          });
    }
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;
      newStdDevMode = "Single (No tags, no est pose, high)";
    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        // if (tgt.poseAmbiguity < 0.4) {
        // avgDist += 2;
        // continue;
        // }
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
        newStdDevMode = "Single (No tags, low)";
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        stdevMode = "Using singletag devs";
        if (numTags > 1) {
          estStdDevs = kMultiTagStdDevs;
          stdevMode = "Using multitag stdDevs";
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > distanceCutoffMeters) {
          newStdDevMode = "Cutoff (one tag, long)";
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          newStdDevMode = "Many, distance augmented (one tag, long)";
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30)).times(1.1);
        }
        curStdDevs = estStdDevs;
      }
    }
  }

  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    parentTable
        .addStructLogger(
            camName + "rawLatestPose", logMode, () -> intRawPose3d.toPose2d(), Pose2d.struct)
        .addStructLogger(camName + "camPose", logMode, () -> robotToCam, Transform3d.struct)
        .addDoubleLogger(camName + "latency", logMode, () -> latency)
        .addStringLogger(camName + "curStdDevs", logMode, () -> curStdDevs.toString())
        .addStringLogger(camName + "stdDevMode", logMode, () -> stdevMode)
        .addStringLogger(camName + "newStdDevMode", logMode, () -> newStdDevMode)
        .addStringLogger(camName + "currentStrat", logMode, () -> currentStrat)
        .addBooleanLogger(camName + "multiTagFound", logMode, () -> multiTagFound);
  }
}
