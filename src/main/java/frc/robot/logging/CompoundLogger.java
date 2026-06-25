package frc.robot.logging;

public interface CompoundLogger {
  public default String getName() {
    return getClass().getName();
  }

  public Object getOperatingObject();

  public default void update() {}

  public abstract LogMode getLogMode();
}
