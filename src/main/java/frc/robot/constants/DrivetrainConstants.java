package frc.robot.constants;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Robot;

public class DrivetrainConstants {
  public static final SwerveRequest.Idle IDLE = new SwerveRequest.Idle();

  public static final SwerveRequest.FieldCentricFacingAngle DRIVE_FIELD_ROT_LOCK;

  static {
    var tmpRequest =
        new SwerveRequest.FieldCentricFacingAngle().withDeadband(0.1).withRotationalDeadband(0.1);

    tmpRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    if (Robot.isReal()) {
      tmpRequest = tmpRequest.withHeadingPID(1.5, 0, 0);
    } else {
      tmpRequest = tmpRequest.withHeadingPID(2.5, 0, 0);
    }

    DRIVE_FIELD_ROT_LOCK = tmpRequest;
  }

  public static final SwerveRequest.FieldCentric DRIVE_FIELD =
      new SwerveRequest.FieldCentric().withDeadband(0.09).withRotationalDeadband(0.09);
}
