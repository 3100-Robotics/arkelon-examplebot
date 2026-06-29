package frc.robot.logging.primarylogger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import frc.robot.logging.LogMode;

public final class PrimaryStringLog
    extends PrimaryLogPublisher<String, StringLogEntry, StringPublisher> {
  public PrimaryStringLog(
      String key, LogMode logMode, NetworkTableInstance ntInstance, DataLog dataLog) {
    super(key, logMode, ntInstance, dataLog);
    setLogEntry(new StringLogEntry(dataLog, key));
    setPublisher(ntInstance.getStringTopic(key).publish());
  }

  @Override
  protected void updateFile(String newValue) {
    getLogEntry().append(newValue);
  }

  @Override
  protected void updateNetwork(String newValue) {
    getPublisher().accept(newValue);
  }
}
