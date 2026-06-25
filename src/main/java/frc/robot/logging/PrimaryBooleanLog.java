package frc.robot.logging;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;

public final class PrimaryBooleanLog
    extends PrimaryLogPublisher<Boolean, BooleanLogEntry, BooleanPublisher> {
  public PrimaryBooleanLog(
      String key, LogMode logMode, NetworkTableInstance ntInstance, DataLog dataLog) {
    super(key, logMode, ntInstance, dataLog);
    setLogEntry(new BooleanLogEntry(dataLog, key));
    setPublisher(ntInstance.getBooleanTopic(key).publish());
  }

  @Override
  protected void updateFile(Boolean newValue) {
    getLogEntry().append(newValue);
  }

  @Override
  protected void updateNetwork(Boolean newValue) {
    getPublisher().accept(newValue);
  }
}
