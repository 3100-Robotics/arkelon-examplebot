package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public interface IndexerConstants {
  int motorLowCanID = 40;
  int motorMiddleCanID = 41;
  int motorHighCanID = 42;

  DCMotor lowMotorPhysical = DCMotor.getKrakenX60(1);
  DCMotor midMotorPhysical = DCMotor.getNEO(1);
  DCMotor highMotorPhysical = DCMotor.getNEO(1);

  SmartMotorControllerConfig lowMotorConfig =
      new SmartMotorControllerConfig()
          // Direction / Current stuff
          .withMotorInverted(false)
          .withStatorCurrentLimit(Amps.of(80))
          .withSupplyCurrentLimit(Amps.of(40))
          // Gearing
          .withGearing(1.2) // 24/20 // TODO Learn Gear Ratios
          // Real PID / FF
          .withClosedLoopController(new PIDController(0.01, 0, 0))
          .withFeedforward(new SimpleMotorFeedforward(0, 0.15))
          // Sim PID / FF
          .withSimClosedLoopController(
              new PIDController(0.6, 0, 0)) // new PIDController(0.73, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.145))
          // Control mode
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withTelemetry("lowMotor", TelemetryVerbosity.LOW)
          // Sim props
          .withMomentOfInertia(KilogramSquareMeters.of(0.001));

  SmartMotorControllerConfig midMotorConfig =
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
          .withTelemetry("midMotor", TelemetryVerbosity.LOW)
          // Sim props
          .withMomentOfInertia(KilogramSquareMeters.of(0.001));

  SmartMotorControllerConfig highMotorConfig =
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
          .withTelemetry("highMotor", TelemetryVerbosity.LOW)
          // Sim props
          .withMomentOfInertia(KilogramSquareMeters.of(0.001));
}
