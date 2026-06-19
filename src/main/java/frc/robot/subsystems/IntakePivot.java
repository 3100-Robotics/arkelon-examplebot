package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Targets.IntakePivotTarget;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakePivot extends SubsystemBase {
  interface Const extends IntakeConstants {}

  // Define any special variables to track the current state of the subsystem
  private IntakePivotTarget state =
      IntakePivotTarget.Low; // TODO: Extratc out the default to constants

  // Define vendor motors
  private final TalonFX rawMotor = new TalonFX(Constants.CANIDs.Intake.motorPivot);

  // Define SmartMotorControllers
  private final SmartMotorController motor =
      new TalonFXWrapper(
          rawMotor, Const.pivotMotorPhysical, Const.pivotMotorConfig.withSubsystem(this));

  public void setState(IntakePivotTarget state) {
    this.state = state;
    motor.setPosition(this.state.angle);
  }

  public IntakePivotTarget getState() {
    return state;
  }

  // @Override
  // public void initSendable(SendableBuilder builder) {

  // }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    SmartDashboard.putString(getName() + "/state", state.toString());
    motor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motor.simIterate();
  }
}
