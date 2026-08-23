package frc.robot.sotm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.sbdc.loggerhead.util.LightSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.MatchContext;
import frc.robot.subsystems.Drivetrain;

public class SOTMState extends LightSubsystem {
  private final Drivetrain drivetrain;

  private double flywheelRPM = 0;
  private double hoodAngleDegrees = 0;
  private Rotation2d heading = Rotation2d.kZero;

  public SOTMState(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    var state = drivetrain.getState();
    var solution = SOTMHelper.solve(state.Pose, state.Speeds);
    this.flywheelRPM = solution.flywheelRpm();
    this.heading = solution.aimHeading();
    this.hoodAngleDegrees = solution.hoodAngleDeg();
  }

  public AngularVelocity getFlywheelSpeed() {
    return RPM.of(flywheelRPM);
  }

  public Angle getHoodAngle() {
    return Degrees.of(hoodAngleDegrees);
  }

  public Rotation2d getHeading() {
    return MatchContext.getInstance().isRedAlliance ? heading.plus(Rotation2d.kPi) : heading;
  }
}
