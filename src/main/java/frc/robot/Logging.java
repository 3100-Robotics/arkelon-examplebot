package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.logging.LogXboxController;
import java.util.ArrayList;
import java.util.List;

public class Logging {
  private static Logging INSTANCE = new Logging();

  public static Logging getInstance() {
    return INSTANCE;
  }

  private List<LogXboxController> controllerLogs = new ArrayList<>();

  private Logging() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);
  }

  public void setupControllerLog(String name, Integer slot, CommandXboxController controller) {
    controllerLogs.add(new LogXboxController(name, slot, controller));
  }

  public void periodic() {
    for (LogXboxController logController : controllerLogs) {
      var e = Timer.getFPGATimestamp();
      logController.periodic();
      SmartDashboard.putNumber("ctrlTime", Timer.getFPGATimestamp() - e);
    }
  }
}
