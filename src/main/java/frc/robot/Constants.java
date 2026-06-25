package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

/* TODO LTPS
 *
 * The constants file is where all the constants go.
 * Anything like:
 *  * motor/encoder configurations
 *  * CANIds, limit switch or other DIO channels, PWM Channels
 *  * PID Controllers / feedforward controllers
 */
public class Constants {
  // Telemetry verbosities all go together instead of being in thier specific superstructure
  // sections [LTPS TODO]
  public static interface TelemetryConfigs {
    SmartMotorControllerTelemetryConfig rootConfig =
        new SmartMotorControllerTelemetryConfig().withTelemetryVerbosity(TelemetryVerbosity.LOW);
    SmartMotorControllerTelemetryConfig shooterMotorHood = rootConfig;
    SmartMotorControllerTelemetryConfig shooterMotorLeftFlywheel = rootConfig;
    SmartMotorControllerTelemetryConfig shooterMotorRightFlywheel = rootConfig;

    SmartMotorControllerTelemetryConfig indexerMotorLow = rootConfig;
    SmartMotorControllerTelemetryConfig indexerMotorMid = rootConfig;
    SmartMotorControllerTelemetryConfig indexerMotorHigh = rootConfig;

    SmartMotorControllerTelemetryConfig intakeMotorPivot = rootConfig;
    SmartMotorControllerTelemetryConfig intakeMotorRoller = rootConfig;
  }

  // CAN Ids all go together instead of being in thier specific superstructure sections [LTPS
  // TODO]
  public static interface CANIDs {
    public interface Drivetrain {}

    public interface Shooter {
      int motorHood = 50;
      int motorLeftFlywheel = 51;
      int motorRightFlywheel = 52;
    }

    public interface Intake {
      int motorPivot = 30;
      int motorRoller = 31;
    }

    public interface Indexer {
      int motorLow = 40;
      int motorMiddle = 41;
      int motorHigh = 42;
    }
  }

  public static interface DrivetrainConstants {}

  public static interface ShooterConstants {
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
            .withTelemetry("hoodMotor", TelemetryConfigs.shooterMotorHood)
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
            .withTelemetry("leftFLywheelMotor", TelemetryConfigs.shooterMotorLeftFlywheel)
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
            .withTelemetry("rightFLywheelMotor", TelemetryConfigs.shooterMotorRightFlywheel)
            // Sim props
            .withMomentOfInertia(Inches.of(14.724154), Pound.of(7.8858569));
  }

  public static interface IndexerConstants {
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
            .withTelemetry("lowMotor", TelemetryConfigs.indexerMotorLow)
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
            .withTelemetry("midMotor", TelemetryConfigs.indexerMotorMid)
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
            .withTelemetry("highMotor", TelemetryConfigs.indexerMotorHigh)
            // Sim props
            .withMomentOfInertia(KilogramSquareMeters.of(0.001));
  }

  public static interface IntakeConstants {
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
            .withTelemetry("pivotMotor", TelemetryConfigs.intakeMotorPivot)
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
            .withTelemetry("rollerMotor", TelemetryConfigs.intakeMotorRoller)
            // Sim props
            .withMomentOfInertia(Inches.of(0.675), Pounds.of(0.9136709));
  }
}
