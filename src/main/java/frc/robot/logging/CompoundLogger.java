package frc.robot.logging;

public interface CompoundLogger {
  public default String getName() {
    return getClass().getName() + " " + ((Integer) hashCode()).toString();
  }

  public default void update() {}

  public abstract LogMode getLogMode();
}
