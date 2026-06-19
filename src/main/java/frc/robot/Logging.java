package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Logging {
  private static Logging INSTANCE = new Logging();

  public static Logging getInstance() {
    return INSTANCE;
  }

  private Logging() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);
  }
}
