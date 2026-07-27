package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import frc.robot.Targets.IntakePivotTarget;
import frc.robot.constants.IntakeConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakePivot extends LightSubsystem implements Loggable {
  // Define any special variables to track the current state of the subsystem
  private IntakePivotTarget target =
      IntakePivotTarget.Low; // TODO: Extract out the default to constants

  // Define vendor motors
  private final TalonFX rawMotor = new TalonFX(IntakeConstants.motorPivotCanID);
  private final CANcoder rawEncoder = new CANcoder(IntakeConstants.encoderPivotCanID);

  // Define SmartMotorControllers
  private final SmartMotorController motor =
      new TalonFXWrapper(
          rawMotor,
          IntakeConstants.pivotMotorPhysical,
          IntakeConstants.pivotMotorConfig
              .clone()
              .withExternalEncoder(rawEncoder)
              .withSubsystem(this));

  public void setState(IntakePivotTarget state) {
    this.target = state;
    motor.setPosition(this.target.angle);
  }

  public IntakePivotTarget getState() {
    return target;
  }

  @Override
  public void periodic() {
    motor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motor.simIterate();
  }

  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    parentTable
      .addDoubleLogger("rollerMechRPM", logMode, () -> motor.getMechanismVelocity().in(RPM))
      .addStringLogger("state", logMode, target::toString)
    ;
  }
}
