package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class DrivePointAtPose extends Command {
  private final Drivetrain drivetrain;
  private final Supplier<Double> xSpeed;
  private final Supplier<Double> ySpeed;
  // private Supplier<Double> thetaSpeed;
  private final Supplier<Double> throttle;

  private final Supplier<Pose2d> targetSupplier;

  public DrivePointAtPose(
      Drivetrain drivetrain,
      Supplier<Double> xSpeed,
      Supplier<Double> ySpeed,
      Supplier<Double> throttle,
      Supplier<Pose2d> targetSupplier) {
    this.drivetrain = drivetrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.throttle = throttle;
    this.targetSupplier = targetSupplier;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var robotPose = drivetrain.getState().Pose;
    var targetPose = targetSupplier.get();
    var differencePose = robotPose.minus(targetPose);
    var targetRotation =
        Rotation2d.fromRadians(Math.atan2(differencePose.getY(), differencePose.getX()));

    drivetrain.setControl(
        DrivetrainConstants.DRIVE_FIELD_ROT_LOCK
            .withVelocityX(-xSpeed.get() * throttle.get())
            .withVelocityY(-ySpeed.get() * throttle.get())
            .withTargetDirection(targetRotation));
  }

  @Override
  public void end(boolean inturupted) {
    drivetrain.setControl(DrivetrainConstants.IDLE);
  }
}
