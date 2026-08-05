package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {
  public final AprilTagFieldLayout tagLayout;
  public final Transform3d robotToCam;
  public final String camName;
  public final PhotonCamera camera;
  public final PhotonPoseEstimator photonEstimator;

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 1);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1, 1, 1);

  private Field2d simField;

  public SimCameraProperties simCamProps;
  public PhotonCameraSim sim;
  private Matrix<N3, N1> curStdDevs = VecBuilder.fill(0, 0, 0);

  private String stdevMode = "";
  private double avgDist = 0;
  private Pose3d intRawPose3d = Pose3d.kZero;

  public Camera(
      AprilTagFieldLayout atagfieldlayout,
      Transform3d robotCameraTransform,
      String camName,
      boolean doSim,
      SimCameraProperties simCamProps,
      boolean simDrawWireframe,
      boolean processedStream) {

    tagLayout = atagfieldlayout;
    robotToCam = robotCameraTransform;
    this.camName = camName;
    camera = new PhotonCamera(this.camName);
    photonEstimator = new PhotonPoseEstimator(this.tagLayout, this.robotToCam);

    if (doSim) {
      this.simCamProps = simCamProps;
      this.sim = new PhotonCameraSim(this.camera, this.simCamProps);
      this.sim.enableDrawWireframe(simDrawWireframe);
      this.sim.enableProcessedStream(processedStream);
      this.sim.enableRawStream(processedStream);
    }
  }

  public void setSimField(Field2d simField) {
    this.simField = simField;
  }

  public void update() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : camera.getAllUnreadResults()) {
      visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
      }
      updateEstimationStdDevs(visionEst, result.getTargets());

      if (Robot.isSimulation() && VisionConstants.simulateCoproc) {
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

            intRawPose3d = est.estimatedPose;
            // estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
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
      stdevMode = "#0";
    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
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
        stdevMode = "#1";
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        stdevMode = "#2";
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
          estStdDevs = kMultiTagStdDevs;
          stdevMode = "#3 Multi";
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 3.1) {
          stdevMode = "#4";
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          stdevMode = "#5";
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        curStdDevs = estStdDevs;
      }
    }
  }
}
