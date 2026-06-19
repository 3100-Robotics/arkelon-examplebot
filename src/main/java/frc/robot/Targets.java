package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Optional;

public interface Targets {
  public enum IntakePivotTarget {
    High(120),
    Medium(55),
    Low(0);

    public final Angle angle;

    private IntakePivotTarget(Angle angle) {
      this.angle = angle;
    }

    private IntakePivotTarget(double angle) {
      this.angle = Degrees.of(angle);
    }
  }

  public enum IntakeRollerTarget {
    Forward(4000),
    Reverse(-4000),
    Off;

    public final Optional<AngularVelocity> speed;

    private IntakeRollerTarget(double speed) {
      this.speed = Optional.of(RPM.of(speed));
    }

    private IntakeRollerTarget(
        AngularVelocity lowSpeed, AngularVelocity midSpeed, AngularVelocity highSpeed) {
      this.speed = Optional.of(lowSpeed);
    }

    private IntakeRollerTarget() {
      this.speed = Optional.empty();
    }
  }

  public enum IndexerTarget {
    Forward(1000, 1000, 3000),
    Reverse(-2000, -1000, -3000),
    Off;

    public final Optional<AngularVelocity> lowSpeed;
    public final Optional<AngularVelocity> midSpeed;
    public final Optional<AngularVelocity> highSpeed;

    private IndexerTarget(double lowSpeed, double midSpeed, double highSpeed) {
      this.lowSpeed = Optional.of(RPM.of(lowSpeed));
      this.midSpeed = Optional.of(RPM.of(midSpeed));
      this.highSpeed = Optional.of(RPM.of(highSpeed));
    }

    private IndexerTarget(
        AngularVelocity lowSpeed, AngularVelocity midSpeed, AngularVelocity highSpeed) {
      this.lowSpeed = Optional.of(lowSpeed);
      this.midSpeed = Optional.of(midSpeed);
      this.highSpeed = Optional.of(highSpeed);
    }

    private IndexerTarget() {
      this.lowSpeed = Optional.empty();
      this.midSpeed = Optional.empty();
      this.highSpeed = Optional.empty();
    }
  }
}
