package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Targets.IndexerTarget;
import frc.robot.constants.IndexerConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Indexer extends SubsystemBase {
  // Define vendor motors
  private final TalonFX rawMotorLow = new TalonFX(IndexerConstants.motorLowCanID);
  private final SparkMax rawMotorMiddle =
      new SparkMax(IndexerConstants.motorMiddleCanID, MotorType.kBrushless);
  private final SparkMax rawMotorHigh =
      new SparkMax(IndexerConstants.motorHighCanID, MotorType.kBrushless);

  // Define SmartMotorControllers
  private final SmartMotorController motorLow =
      new TalonFXWrapper(
          rawMotorLow,
          IndexerConstants.lowMotorPhysical,
          IndexerConstants.lowMotorConfig.withSubsystem(this));
  private final SmartMotorController motorMid =
      new SparkWrapper(
          rawMotorMiddle,
          IndexerConstants.midMotorPhysical,
          IndexerConstants.midMotorConfig.withSubsystem(this));
  private final SmartMotorController motorHigh =
      new SparkWrapper(
          rawMotorHigh,
          IndexerConstants.highMotorPhysical,
          IndexerConstants.highMotorConfig.withSubsystem(this));

  // Define state set command
  public void setState(IndexerTarget state) {
    switch (state) {
      case Forward:
      case Reverse:
        motorLow.setVelocity(state.lowSpeed.get());
        motorMid.setVelocity(state.midSpeed.get());
        motorHigh.setVelocity(state.highSpeed.get());
        break;
      case Off:
        motorLow.setDutyCycle(0);
        motorMid.setDutyCycle(0);
        motorHigh.setDutyCycle(0);
        break;
      default:
        break;
    }
  }

  // Define periodics
  @Override
  public void periodic() {
    SmartDashboard.putNumber("indexerLow", motorLow.getMechanismVelocity().in(RPM));

    motorLow.updateTelemetry();
    motorMid.updateTelemetry();
    motorHigh.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    motorLow.simIterate();
    motorMid.simIterate();
    motorHigh.simIterate();
  }
}
