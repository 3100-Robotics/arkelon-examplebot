package frc.robot.vision;

import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.constants.VisionAndPoseEstConstants;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.VisionSystemSim;

public class MainVision extends LightSubsystem implements Loggable {
  public static final AprilTagFieldLayout kTagLayout = VisionAndPoseEstConstants.kTagLayout;

  // Simulation
  private VisionSystemSim visionSim;
  private Supplier<Pose2d> simDTGetter;

  private List<Camera> cameras = new ArrayList<>();

  /**
   * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
   *     org.wpilib.math.estimator.SwerveDrivePoseEstimator}
   */
  @SuppressWarnings("unused")
  public MainVision(
      EstimateConsumer estConsumer, Supplier<Pose2d> simDTGetter, Drivetrain drivetrain) {
    this.simDTGetter = simDTGetter;

    // cameras.add(VisionConstants.CAM_EVAN);
    cameras.add(VisionAndPoseEstConstants.CAM_LEFT);
    cameras.add(VisionAndPoseEstConstants.CAM_RIGHT);

    for (Camera camera : cameras) {
      camera.setPoseOutput(estConsumer);
      camera.setHeadingSupplier(
          () -> {
            return Pair.of(obtainDrivetrainRotation(drivetrain), Timer.getFPGATimestamp());
          });
    }

    if ((!Robot.isReal()) && VisionAndPoseEstConstants.simulateCoproc) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(kTagLayout);
      for (Camera camera : cameras) {
        visionSim.addCamera(camera.sim, camera.robotToCam);
        camera.setSimField(getSimDebugField());
      }
    }
  }

  public Rotation3d obtainDrivetrainRotation(Drivetrain drivetrain) {
    if (Robot.isReal()) {
      return drivetrain.getPigeon2().getRotation3d();
    } else {
      // return new Rotation3d(0,0, drivetrain.getPigeon2().getRotation2d());
      return new Rotation3d(drivetrain.getPigeon2().getRotation2d());
    }
  }

  @Override
  public void periodic() {
    for (Camera camera : cameras) {
      camera.update();
    }

    if ((!Robot.isReal()) && VisionAndPoseEstConstants.simulateCoproc) {
      visionSim.update(simDTGetter.get());
    }
  }

  /** Reset pose history of the robot in the vision system simulation. */
  @SuppressWarnings("unused")
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation() && VisionAndPoseEstConstants.simulateCoproc)
      visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    if (!VisionAndPoseEstConstants.simulateCoproc) return null;
    return visionSim.getDebugField();
  }

  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    for (Camera camera : cameras) {
      parentTable.addLoggable(camera, logMode);
    }
  }

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
