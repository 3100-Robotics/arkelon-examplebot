package frc.robot.logging;

public interface RunsPeriodic {
  public default void periodic() {}
  ;

  public default String getName() {
    return this.getClass().toString() + " " + this.hashCode();
  }
  ;
}
