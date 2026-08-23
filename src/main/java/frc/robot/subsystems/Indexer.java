package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.sbdc.loggerhead.logging.LogMode;
import com.sbdc.loggerhead.logging.Loggable;
import com.sbdc.loggerhead.logging.Loggerhead;
import com.sbdc.loggerhead.logging.Table;
import com.sbdc.loggerhead.util.LightSubsystem;
import frc.robot.Targets.IndexerTarget;
import frc.robot.constants.IndexerConstants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Indexer extends LightSubsystem implements Loggable {
  // Define vendor motors
  private final TalonFX rawMotorLow = new TalonFX(IndexerConstants.motorLowCanID);
  private final SparkMax rawMotorMiddle =
      new SparkMax(IndexerConstants.motorMiddleCanID, MotorType.kBrushless);
  private final SparkMax rawMotorHigh =
      new SparkMax(IndexerConstants.motorHighCanID, MotorType.kBrushless);

  private IndexerTarget target = IndexerTarget.Off;

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
    this.target = state;
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

  public void setupLogging(Table parentTable, LogMode logMode, Loggerhead loggerhead) {
    parentTable.addDoubleLogger(
        "lowMechansimRPM", logMode, () -> motorLow.getMechanismVelocity().in(RPM));
    parentTable.addDoubleLogger(
        "midMechansimRPM", logMode, () -> motorMid.getMechanismVelocity().in(RPM));
    parentTable.addDoubleLogger(
        "highMechansimRPM", logMode, () -> motorHigh.getMechanismVelocity().in(RPM));
    parentTable.addStringLogger("target", logMode, () -> target.toString());
  }
}
