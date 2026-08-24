package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.sbdc.loggerhead.logging.LogMode;
import com.sbdc.loggerhead.logging.Loggable;
import com.sbdc.loggerhead.logging.Loggerhead;
import com.sbdc.loggerhead.logging.Table;
import com.sbdc.loggerhead.util.LightSubsystem;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ShooterConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class Flywheels extends LightSubsystem implements Loggable {
  // Define vendor motors
  private final SparkMax rawMotorLeftFlywheel =
      new SparkMax(ShooterConstants.motorLeftFlywheelCanID, MotorType.kBrushless);
  private final SparkMax rawMotorRightFlywheel =
      new SparkMax(ShooterConstants.motorRightFlywheelCanID, MotorType.kBrushless);

  // Define SmartMotorControllers
  private final SmartMotorController motorLeftFlywheel =
      new SparkWrapper(
          rawMotorLeftFlywheel,
          ShooterConstants.leftFlywheelMotorPhysical,
          ShooterConstants.leftFlywheelMotorConfig.withSubsystem(this));
  private final SmartMotorController motorRightFlywheel =
      new SparkWrapper(
          rawMotorRightFlywheel,
          ShooterConstants.rightFlywheelMotorPhysical,
          ShooterConstants.rightFlywheelMotorConfig.withSubsystem(this));

  public void setSpeed(AngularVelocity speed) {
    motorLeftFlywheel.setVelocity(speed);
    motorRightFlywheel.setVelocity(speed);
  }

  public void stop() {
    motorLeftFlywheel.setDutyCycle(0);
    motorRightFlywheel.setDutyCycle(0);
  }

  @Override
  public void periodic() {
    motorLeftFlywheel.updateTelemetry();
    motorRightFlywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motorLeftFlywheel.simIterate();
    motorRightFlywheel.simIterate();
  }

  @Override
  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    parentTable
        .addDoubleLogger(
            "leftMechVelocity", logMode, () -> motorLeftFlywheel.getMechanismVelocity().in(RPM))
        .addDoubleLogger(
            "rightMechVelocity", logMode, () -> motorRightFlywheel.getMechanismVelocity().in(RPM));
  }
}
