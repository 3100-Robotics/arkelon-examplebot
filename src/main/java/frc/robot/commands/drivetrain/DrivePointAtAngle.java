package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class DrivePointAtAngle extends Command {
  private final Drivetrain drivetrain;
  private final Supplier<Double> xSpeed;
  private final Supplier<Double> ySpeed;
  private final Supplier<Double> throttle;

  private final Supplier<Rotation2d> targetSupplier;

  public DrivePointAtAngle(
      Drivetrain drivetrain,
      Supplier<Double> xSpeed,
      Supplier<Double> ySpeed,
      Supplier<Double> throttle,
      Supplier<Rotation2d> targetSupplier) {
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
    drivetrain.setControl(
        DrivetrainConstants.DRIVE_FIELD_ROT_LOCK
            .withVelocityX(-xSpeed.get() * throttle.get())
            .withVelocityY(-ySpeed.get() * throttle.get())
            .withTargetDirection(this.targetSupplier.get()));
  }

  @Override
  public void end(boolean inturupted) {
    drivetrain.setControl(DrivetrainConstants.IDLE);
  }
}
