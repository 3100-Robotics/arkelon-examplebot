package frc.robot.logging;

public interface GenericLoggable {
  public default void setupLogging(
      String subsystemRoot, LogMode logMode, RootLogging rootLogging) {}
  ;
}
