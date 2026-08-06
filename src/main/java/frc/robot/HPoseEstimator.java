package frc.robot;

import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.Drivetrain;

public class HPoseEstimator extends LightSubsystem implements Loggable {
  private final SwerveDrivePoseEstimator est;

  public final Drivetrain drivetrain;

  public HPoseEstimator(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    est =
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
    est.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    est.update(drivetrain.getPigeon2().getRotation2d(), drivetrain.getModulePositions());
  }

  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    parentTable.addPoseLogger("estimatedPose", logMode, est::getEstimatedPosition);
  }
}
