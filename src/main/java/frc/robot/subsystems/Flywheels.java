package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

public class Flywheels extends SubsystemBase {
  // Define vendor motors
  private final SparkMax rawMotorLeftFlywheel =
      new SparkMax(Constants.CANIDs.Shooter.motorLeftFlywheel, MotorType.kBrushless);
  private final SparkMax rawMotorRightFlywheel =
      new SparkMax(Constants.CANIDs.Shooter.motorRightFlywheel, MotorType.kBrushless);

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
    SmartDashboard.putData(this);
    motorLeftFlywheel.updateTelemetry();
    motorRightFlywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motorLeftFlywheel.simIterate();
    motorRightFlywheel.simIterate();
  }
}
