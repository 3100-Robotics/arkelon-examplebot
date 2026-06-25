package frc.robot.logging;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.IntegerLogEntry;

public final class PrimaryIntegerLog
    extends PrimaryLogPublisher<Integer, IntegerLogEntry, IntegerPublisher> {
  public PrimaryIntegerLog(
      String key, LogMode logMode, NetworkTableInstance ntInstance, DataLog dataLog) {
    super(key, logMode, ntInstance, dataLog);
    setLogEntry(new IntegerLogEntry(dataLog, key));
    setPublisher(ntInstance.getIntegerTopic(key).publish());
  }

  @Override
  protected void updateFile(Integer newValue) {
    getLogEntry().append(newValue);
  }

  @Override
  protected void updateNetwork(Integer newValue) {
    getPublisher().accept(newValue);
  }
}
