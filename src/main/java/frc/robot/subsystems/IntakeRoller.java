package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.sbdc.loggerhead.LightSubsystem;
import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggable;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.Table;
import frc.robot.Targets.IntakeRollerTarget;
import frc.robot.constants.IntakeConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeRoller extends LightSubsystem implements Loggable {
  private IntakeRollerTarget target = IntakeRollerTarget.Off;

  // Define vendor motors
  private final SparkMax rawMotor =
      new SparkMax(IntakeConstants.motorRollerCanID, MotorType.kBrushless);

  // Define SmartMotorControllers
  private final SmartMotorController motor =
      new SparkWrapper(
          rawMotor,
          IntakeConstants.pivotMotorPhysical,
          IntakeConstants.rollerMotorConfig.withSubsystem(this));

  // Define state set command
  public void setState(IntakeRollerTarget state) {
    this.target = state;
    switch (state) {
      case Forward:
      case Reverse:
        motor.setVelocity(state.speed.get());
        break;
      case Off:
        motor.setDutyCycle(0);
        break;
      default:
        break;
    }
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
