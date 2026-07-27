package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public interface IndexerConstants {
  int motorLowCanID = 40;
  int motorMiddleCanID = 41;
  int motorHighCanID = 42;

  DCMotor lowMotorPhysical = DCMotor.getKrakenX60(1);
  DCMotor midMotorPhysical = DCMotor.getNEO(1);
  DCMotor highMotorPhysical = DCMotor.getNEO(1);

  SmartMotorControllerConfig baseRollerConfig =
      new SmartMotorControllerConfig()
          // Direction / Current stuff
          .withStatorCurrentLimit(Amps.of(60))
          .withIdleMode(MotorMode.COAST)
          // .withClosedLoopRampRate(Seconds.of(0.25))
          // .withOpenLoopRampRate(Seconds.of(0.25))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withMomentOfInertia(KilogramSquareMeters.of(0.001));

  SmartMotorControllerConfig lowMotorConfig =
      baseRollerConfig
          .clone()
          // Direction / Current stuff
          .withMotorInverted(true)
          .withSupplyCurrentLimit(Amps.of(40))
          .withGearing(
              new MechanismGearing(GearBox.fromTeeth(24, 20))) // 24/20 // TODO Learn Gear Ratios
          // Real PID / FF
          .withClosedLoopController(new PIDController(0.01, 0, 0))
          .withFeedforward(new SimpleMotorFeedforward(0, 0.15))
          // Sim PID / FF
          .withSimClosedLoopController(
              new PIDController(0.05, 0, 0)) // new PIDController(0.73, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.1))
          .withTelemetry("lowMotor", TelemetryVerbosity.LOW);

  SmartMotorControllerConfig midMotorConfig =
      baseRollerConfig
          .clone()
          .withMotorInverted(false)
          .withGearing(new MechanismGearing(GearBox.fromTeeth(24, 20)))
          // PID / FF
          .withClosedLoopController(new PIDController(0.01, 0, 0))
          .withFeedforward(new SimpleMotorFeedforward(0, 0.12))
          // Sim PID / FF
          .withSimClosedLoopController(new PIDController(0.05, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.1))
          .withTelemetry("midMotor", TelemetryVerbosity.LOW);

  SmartMotorControllerConfig highMotorConfig =
      baseRollerConfig
          .clone()
          .withMotorInverted(false)
          .withGearing(new MechanismGearing(GearBox.fromTeeth(36, 20)))

          // PID / FF
          .withClosedLoopController(new PIDController(0.01, 0, 0))
          .withFeedforward(new SimpleMotorFeedforward(0, 0.2))
          // Sim PID / FF
          .withSimClosedLoopController(new PIDController(0.05, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.1))
          // Control mode
          .withTelemetry("highMotor", TelemetryVerbosity.LOW);
}
