package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;


public interface ShooterConstants {
    int motorHoodCanID = 50;
    int motorLeftFlywheelCanID = 51;
    int motorRightFlywheelCanID = 52;

    DCMotor hoodMotorPhysical = DCMotor.getKrakenX60(1);
    DCMotor leftFlywheelMotorPhysical = DCMotor.getNEO(1);
    DCMotor rightFlywheelMotorPhysical = DCMotor.getNEO(1);

    SmartMotorControllerConfig hoodMotorConfig =
        new SmartMotorControllerConfig()
            // Direction / Current stuff
            .withMotorInverted(false)
            .withStatorCurrentLimit(Amps.of(80))
            .withSupplyCurrentLimit(Amps.of(40))
            // Gearing
            .withGearing(new MechanismGearing(GearBox.fromStages("48:12", "182:10")))
            // PID / FF
            .withClosedLoopController(new PIDController(5, 0, 0.1))
            .withFeedforward(new ArmFeedforward(0, 0, 0))
            // Sim PID / FF
            .withSimClosedLoopController(new PIDController(0, 0, 0))
            .withSimFeedforward(new ArmFeedforward(0, 0, 0))
            // Control mode
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withTelemetry("hoodMotor", TelemetryVerbosity.LOW)
            // Sim props
            .withMomentOfInertia(KilogramSquareMeters.of(0.0190245794))
            // Starting Position
            .withStartingPosition(Degrees.of(12.667292));

    SmartMotorControllerConfig leftFlywheelMotorConfig =
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
            .withTelemetry("leftFLywheelMotor", TelemetryVerbosity.LOW)
            // Sim props
            .withMomentOfInertia(Inches.of(14.724154), Pound.of(7.8858569));

    SmartMotorControllerConfig rightFlywheelMotorConfig =
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
            .withTelemetry("rightFLywheelMotor", TelemetryVerbosity.LOW)
            // Sim props
            .withMomentOfInertia(Inches.of(14.724154), Pound.of(7.8858569));
  }