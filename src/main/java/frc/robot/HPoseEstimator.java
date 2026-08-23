package frc.robot;

import com.sbdc.loggerhead.util.LightSubsystem;
import com.sbdc.loggerhead.logging.LogMode;
import com.sbdc.loggerhead.logging.Loggable;
import com.sbdc.loggerhead.logging.Loggerhead;
import com.sbdc.loggerhead.logging.Table;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.Drivetrain;

public class HPoseEstimator extends LightSubsystem implements Loggable {
  private final SwerveDrivePoseEstimator estWithVision;
  private final SwerveDrivePoseEstimator estNoVision;

  public final Drivetrain drivetrain;

  public HPoseEstimator(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    estWithVision =
        new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            Rotation2d.kZero,
            drivetrain.getModulePositions(),
            Pose2d.kZero);

    estNoVision =
        new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            Rotation2d.kZero,
            drivetrain.getModulePositions(),
            Pose2d.kZero);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // if (visionRobotPoseMeters
    //         .getTranslation()
    //         .getDistance(estWithVision.getEstimatedPosition().getTranslation())
    //     < VisionAndPoseEstConstants.maxReasonableVisionDistance) {
    estWithVision.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);

    // if (Robot.isReal()) {
    drivetrain.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    // }
    // }
  }

  public void reset(Pose2d resetPose, boolean vision, boolean noVision) {
    if (vision) {
      estWithVision.resetPose(resetPose);
    }
    if (noVision) {
      estNoVision.resetPose(resetPose);
    }
  }

  @Override
  public void periodic() {
    estWithVision.update(drivetrain.getPigeon2().getRotation2d(), drivetrain.getModulePositions());
    estNoVision.update(drivetrain.getPigeon2().getRotation2d(), drivetrain.getModulePositions());
  }

  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    parentTable.addStructLogger(
        "estimatedPose", logMode, estWithVision::getEstimatedPosition, Pose2d.struct);
    parentTable.addStructLogger(
        "estimatedPoseNoVision", logMode, estNoVision::getEstimatedPosition, Pose2d.struct);
  }
}
