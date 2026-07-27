package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pound;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.List;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public interface ShooterConstants {
  // Shot table
  public class ShotTable {
    public static final List<Pair<Distance, Double>> distanceAngleTable = new ArrayList<>();
    public static final List<Pair<Distance, Double>> distanceSpeedTable = new ArrayList<>();

    static {
      // Distance Angle
      distanceAngleTable.add(Pair.of(Meters.of(1.36), 20.0 - 2.5)); // Close
      distanceAngleTable.add(Pair.of(Inches.of(135), 27.8)); // Tower fwd
      distanceAngleTable.add(Pair.of(Inches.of(164), 31.5)); // Tower bkwd
      distanceAngleTable.add(Pair.of(Meters.of(5.86), 40.0 - 2.5)); // Far

      // Distance Speed
      distanceSpeedTable.add(Pair.of(Meters.of(1.36), 2400.0 + (2400 * 0.05))); // Close
      distanceSpeedTable.add(Pair.of(Inches.of(135), 3100.0)); // Tower fwd
      distanceSpeedTable.add(Pair.of(Inches.of(164), 3300.0)); // Tower bkwd
      distanceSpeedTable.add(Pair.of(Meters.of(5.86), 3960.0 + (3960 * 0.05))); // Far
    }
  }

  Angle maxHoodAngle = Degrees.of(40);
  Angle minHoodAngle = Degrees.of(12.667292);

  int motorHoodCanID = 50;
  int motorLeftFlywheelCanID = 51;
  int motorRightFlywheelCanID = 53;

  DCMotor hoodMotorPhysical = DCMotor.getKrakenX60(1);
  DCMotor leftFlywheelMotorPhysical = DCMotor.getNEO(1);
  DCMotor rightFlywheelMotorPhysical = DCMotor.getNEO(1);

  SmartMotorControllerConfig hoodMotorConfig =
      new SmartMotorControllerConfig()
          // Direction / Current stuff
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(50))
          .withSupplyCurrentLimit(Amps.of(40))
          // Gearing
          .withGearing(new MechanismGearing(GearBox.fromStages("48:12", "182:10")))
          // PID / FF
          .withClosedLoopController(new PIDController(530, 0, 0))
          .withFeedforward(new ArmFeedforward(140, 0, 0))
          // Sim PID / FF
          .withSimClosedLoopController(new PIDController(5, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          // Control mode
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withTelemetry("hoodMotor", TelemetryVerbosity.LOW)
          // Sim props
          .withMomentOfInertia(KilogramSquareMeters.of(0.0190245794))
          // Starting Position
          .withStartingPosition(ShooterConstants.minHoodAngle);

  SmartMotorControllerConfig baseFlywheelConfig =
      new SmartMotorControllerConfig()
          .withStatorCurrentLimit(Amps.of(60))
          .withIdleMode(MotorMode.COAST)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withGearing(1)
          .withMomentOfInertia(Inches.of(1.9825395), Pound.of(0.9))

      // .withVendorConfig(Constants.MotorConfigs.noBadFilteringNEO) // TODO Implement this
      ;

  SmartMotorControllerConfig leftFlywheelMotorConfig =
      baseFlywheelConfig
          .clone()
          .withMotorInverted(true)
          // PID / FF
          .withClosedLoopController(new PIDController(0.003, 0, 0))
          .withFeedforward(new SimpleMotorFeedforward(0, 0.137))
          // Sim PID / FF
          .withSimClosedLoopController(new PIDController(0.003, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.137))
          // Telemetry
          .withTelemetry("leftFlywheelMotor", TelemetryVerbosity.LOW);

  SmartMotorControllerConfig rightFlywheelMotorConfig =
      baseFlywheelConfig
          .clone()
          .withMotorInverted(false)
          // PID / FF
          .withClosedLoopController(new PIDController(0.003, 0, 0))
          .withFeedforward(new SimpleMotorFeedforward(0, 0.1399))
          // Sim PID / FF
          .withSimClosedLoopController(new PIDController(0.003, 0, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, 0.1399))
          // Control mode
          .withTelemetry("rightFLywheelMotor", TelemetryVerbosity.LOW);
}
