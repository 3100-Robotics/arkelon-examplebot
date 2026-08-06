package frc.robot.vision;

import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.simulation.VisionSystemSim;

public class MainVision extends LightSubsystem implements Loggable {
  public static final AprilTagFieldLayout kTagLayout = VisionConstants.kTagLayout;

  // Simulation
  private VisionSystemSim visionSim;
  private Supplier<Pose2d> simDTGetter;

  private List<Camera> cameras = new ArrayList<>();

  /**
   * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
   *     org.wpilib.math.estimator.SwerveDrivePoseEstimator}
   */
  @SuppressWarnings("unused")
  public MainVision(EstimateConsumer estConsumer, Supplier<Pose2d> simDTGetter) {
    this.simDTGetter = simDTGetter;

    // cameras.add(VisionConstants.CAM_EVAN);
    cameras.add(VisionConstants.CAM_LEFT);
    cameras.add(VisionConstants.CAM_RIGHT);

    for (Camera camera : cameras) {
      camera.setPoseOutput(estConsumer);
    }

    if ((!Robot.isReal()) && VisionConstants.simulateCoproc) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(kTagLayout);
      for (Camera camera : cameras) {
        visionSim.addCamera(camera.sim, camera.robotToCam);
        camera.setSimField(getSimDebugField());
      }
    }
  }

  @Override
  public void periodic() {
    for (Camera camera : cameras) {
      camera.update();
    }

    if ((!Robot.isReal()) && VisionConstants.simulateCoproc) {
      visionSim.update(simDTGetter.get());
    }
  }

  /** Reset pose history of the robot in the vision system simulation. */
  @SuppressWarnings("unused")
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation() && VisionConstants.simulateCoproc) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    if (!VisionConstants.simulateCoproc) return null;
    return visionSim.getDebugField();
  }

  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {}

  @FunctionalInterface
  public static interface EstimateConsumer {
    public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
  }
}
