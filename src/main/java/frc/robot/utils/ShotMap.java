package frc.robot.utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * A ShotMap will provide an angle and a speed for the hood and flywheels with two methods: {@link
 * #getFlywheelSpeed} and {@link #getHoodAngle}. The caller is not responsible for updating the
 * state of the shotMaps model for generating these numbers
 */
public interface ShotMap {
  /**
   * Acts as a descriptor for what this shot map targets. Used primarily by anything that publishes
   * to NetworkTables or that references a command using the shotMap.
   *
   * @return the string describing what this shotMap targets
   */
  public abstract String getTarget();

  /**
   * @return the flywheel speed
   */
  public abstract AngularVelocity getFlywheelSpeed();

  /**
   * @return the hood angle
   */
  public abstract Angle getHoodAngle();
}
