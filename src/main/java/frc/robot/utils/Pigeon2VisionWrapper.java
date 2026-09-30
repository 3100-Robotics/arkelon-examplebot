package frc.robot.utils;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;

public class Pigeon2VisionWrapper {
  public final Pigeon2 pigeon2;
  private Rotation3d offset = Rotation3d.kZero;

  public Pigeon2VisionWrapper(Pigeon2 pigeon2) {
    this.pigeon2 = pigeon2;
  }

  public Rotation3d getRotation() {
    return pigeon2.getRotation3d().minus(offset);
  }

  public void zero() {
    offset = pigeon2.getRotation3d();
  }
}
