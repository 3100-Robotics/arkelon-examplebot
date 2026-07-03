package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team3100.loggerhead.LogMode;
import com.team3100.loggerhead.Loggable;
import com.team3100.loggerhead.Loggerhead;
import com.team3100.loggerhead.Table;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood implements Subsystem, Loggable {
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
    motor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motor.simIterate();
  }

  @Override
  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    parentTable.addDoubleLogger(
        "hoodAngle", logMode, () -> motor.getMechanismPosition().in(Degrees));
  }
}
