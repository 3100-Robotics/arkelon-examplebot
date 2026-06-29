package frc.robot.logging.primarylogger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import frc.robot.logging.LogMode;

public final class PrimarySwerveStateLog
    extends PrimaryLogPublisher<
        SwerveModuleState[],
        StructArrayLogEntry<SwerveModuleState>,
        StructArrayPublisher<SwerveModuleState>> {
  public PrimarySwerveStateLog(
      String key, LogMode logMode, NetworkTableInstance ntInstance, DataLog dataLog) {
    super(key, logMode, ntInstance, dataLog);
    setLogEntry(StructArrayLogEntry.create(dataLog, key, SwerveModuleState.struct));
    setPublisher(ntInstance.getStructArrayTopic(key, SwerveModuleState.struct).publish());
  }

  @Override
  protected void updateFile(SwerveModuleState[] newValue) {
    getLogEntry().append(newValue);
  }

  @Override
  protected void updateNetwork(SwerveModuleState[] newValue) {
    getPublisher().accept(newValue);
  }
}
