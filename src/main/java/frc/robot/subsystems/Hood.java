package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {
  // Define vendor motors
  private final TalonFX rawMotor = new TalonFX(Constants.CANIDs.Shooter.motorHood);

  // Define SmartMotorControllers
  private final SmartMotorController motor =
      new TalonFXWrapper(
          rawMotor,
          ShooterConstants.hoodMotorPhysical,
          ShooterConstants.hoodMotorConfig.withSubsystem(this));

  public void setHoodAngle(Angle angle) {
    motor.setPosition(angle);
  }

  public void stopHood() {
    motor.setDutyCycle(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    SmartDashboard.putNumber(getName() + "/hoodAngle", motor.getMechanismPosition().in(Degrees));
    motor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motor.simIterate();
  }
}
