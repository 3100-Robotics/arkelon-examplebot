package frc.robot.logging.compoundlogger;

import frc.robot.logging.LogMode;

public interface CompoundLogger {
  public default String getName() {
    return getClass().getName();
  }

  public default void update() {}

  public abstract LogMode getLogMode();

  public abstract void initialize(String parentTable);
}
