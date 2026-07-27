package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Targets.IntakePivotTarget;
import frc.robot.constants.IntakeConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakePivot extends SubsystemBase {
  // Define any special variables to track the current state of the subsystem
  private IntakePivotTarget state =
      IntakePivotTarget.Low; // TODO: Extratc out the default to constants

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
    SmartDashboard.putString(getName() + "/state", state.toString());
    motor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motor.simIterate();
  }
}
