package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class DriveTeleop extends Command {
  private Drivetrain drivetrain;
  private Supplier<Double> xSpeed;
  private Supplier<Double> ySpeed;
  private Supplier<Double> thetaSpeed;
  private Supplier<Double> throttle;

  // Requests
  private static final SwerveRequest.Idle IDLE = new SwerveRequest.Idle();
  private static final SwerveRequest.FieldCentric DRIVE_FIELD =
      new SwerveRequest.FieldCentric().withDeadband(0).withRotationalDeadband(0);

  public DriveTeleop(
      Drivetrain drivetrain,
      Supplier<Double> xSpeed,
      Supplier<Double> ySpeed,
      Supplier<Double> thetaSpeed,
      Supplier<Double> throttle) {
    this.drivetrain = drivetrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.thetaSpeed = thetaSpeed;
    this.throttle = throttle;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrain.setControl(
        DRIVE_FIELD
            .withVelocityX(xSpeed.get() * throttle.get())
            .withVelocityY(ySpeed.get() * throttle.get())
            .withRotationalRate(thetaSpeed.get()));
  }

  @Override
  public void end(boolean inturupted) {
    drivetrain.setControl(IDLE);
  }
}
