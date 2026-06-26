package frc.robot.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;

public final class PrimaryPoseLog
    extends PrimaryLogPublisher<Pose2d, StructLogEntry<Pose2d>, StructPublisher<Pose2d>> {
  public PrimaryPoseLog(
      String key, LogMode logMode, NetworkTableInstance ntInstance, DataLog dataLog) {
    super(key, logMode, ntInstance, dataLog);
    setLogEntry(StructLogEntry.create(dataLog, key, Pose2d.struct));
    setPublisher(ntInstance.getStructTopic(key, Pose2d.struct).publish());
  }

  @Override
  protected void updateFile(Pose2d newValue) {
    getLogEntry().append(newValue);
  }

  @Override
  protected void updateNetwork(Pose2d newValue) {
    getPublisher().accept(newValue);
  }
}
