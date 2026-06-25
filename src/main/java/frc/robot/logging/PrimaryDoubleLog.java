package frc.robot.logging;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public final class PrimaryDoubleLog
    extends PrimaryLogPublisher<Double, DoubleLogEntry, DoublePublisher> {
  public PrimaryDoubleLog(
      String key, LogMode logMode, NetworkTableInstance ntInstance, DataLog dataLog) {
    super(key, logMode, ntInstance, dataLog);
    setLogEntry(new DoubleLogEntry(dataLog, key));
    setPublisher(ntInstance.getDoubleTopic(key).publish());
  }

  @Override
  protected void updateFile(Double newValue) {
    getLogEntry().append(newValue);
  }

  @Override
  protected void updateNetwork(Double newValue) {
    getPublisher().accept(newValue);
  }
}
