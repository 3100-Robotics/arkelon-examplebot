package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.vision.Camera;
import org.photonvision.simulation.SimCameraProperties;

public class VisionConstants {
  public static final boolean simulateCoproc = true;

  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final Camera CAM_EVAN =
      new Camera(
          kTagLayout,
          new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0, 0, 0)),
          "sixseven",
          (!Robot.isReal()) && simulateCoproc,
          new SimCameraProperties()
              .setCalibration(960, 720, Rotation2d.fromDegrees(90))
              .setCalibError(0.35, 0.10)
              .setFPS(15)
              .setAvgLatencyMs(50)
              .setLatencyStdDevMs(15),
          true,
          true);
}
