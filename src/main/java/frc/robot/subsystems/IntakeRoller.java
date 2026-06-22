package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Targets.IntakeRollerTarget;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeRoller extends SubsystemBase {
  // Define vendor motors
  private final SparkMax rawMotor =
      new SparkMax(Constants.CANIDs.Intake.motorRoller, MotorType.kBrushless);

  // Define SmartMotorControllers
  private final SmartMotorController motor =
      new SparkWrapper(
          rawMotor,
          IntakeConstants.pivotMotorPhysical,
          IntakeConstants.rollerMotorConfig.withSubsystem(this));

  // Define state set command
  public void setState(IntakeRollerTarget state) {
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
    SmartDashboard.putData(this);
    motor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motor.simIterate();
  }
}
