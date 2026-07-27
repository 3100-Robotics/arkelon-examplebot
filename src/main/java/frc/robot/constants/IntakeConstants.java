package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public interface IntakeConstants {
  int motorPivotCanID = 30;
  int motorRollerCanID = 31;

  DCMotor pivotMotorPhysical = DCMotor.getKrakenX60(1);
  DCMotor rollerMotorPhysical = DCMotor.getNEO(1);

  SmartMotorControllerConfig pivotMotorConfig =
      new SmartMotorControllerConfig()
          // Direction / Current stuff
          .withMotorInverted(false)
          .withStatorCurrentLimit(Amps.of(80))
          .withSupplyCurrentLimit(Amps.of(40))
          // Gearing
          .withGearing(1)
          // PID / FF
          .withClosedLoopController(new PIDController(5, 0, 0.1))
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          // Sim PID / FF
          .withSimClosedLoopController(new PIDController(5, 0, 0.1))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          // Control mode
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withTelemetry("pivotMotor", TelemetryVerbosity.LOW)
          // Sim props
          .withMomentOfInertia(Inches.of(14.724154), Pound.of(7.8858569));

  SmartMotorControllerConfig rollerMotorConfig =
      new SmartMotorControllerConfig()
          // Direction / Current stuff
          .withMotorInverted(false)
          .withStatorCurrentLimit(Amps.of(80))
          .withSupplyCurrentLimit(Amps.of(40))
          // Gearing
          .withGearing(1)
          // PID / FF
          .withClosedLoopController(new PIDController(5, 0, 0.1))
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          // Sim PID / FF
          .withSimClosedLoopController(new PIDController(5, 0, 0.1))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          // Control mode
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withTelemetry("rollerMotor", TelemetryVerbosity.LOW)
          // Sim props
          .withMomentOfInertia(Inches.of(0.675), Pounds.of(0.9136709));
}
