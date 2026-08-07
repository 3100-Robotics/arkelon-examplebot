package frc.robot.utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.sotm.SOTMState;

public class DynamicShotMap implements ShotMap {

  private final SOTMState sotmstate;

  public DynamicShotMap(SOTMState sotmstate) {
    this.sotmstate = sotmstate;
  }

  public String getTarget() {
    return "Dynamic SOTM";
  }

  public AngularVelocity getFlywheelSpeed() {
    return sotmstate.getFlywheelSpeed();
  }

  public Angle getHoodAngle() {
    return sotmstate.getHoodAngle();
  }
}
