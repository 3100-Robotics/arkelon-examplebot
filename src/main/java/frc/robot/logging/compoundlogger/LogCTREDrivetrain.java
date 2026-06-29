package frc.robot.logging.compoundlogger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import frc.robot.logging.LogMode;
import frc.robot.logging.OneShot;
import frc.robot.logging.RootLogging;

public class LogCTREDrivetrain implements CompoundLogger {
  private final SwerveDrivetrain<?, ?, ?> drivetrain;
  private final LogMode logMode;
  private final String name;

  public LogCTREDrivetrain(String name, LogMode logMode, SwerveDrivetrain<?, ?, ?> drivetrain) {
    this.name = name;
    this.drivetrain = drivetrain;
    this.logMode = logMode;
  }

  @Override
  public void initialize(String parentTable) {
    String logRootSwerve = parentTable + name + "/Swerve/";
    String logRootPose = parentTable + name + "/Pose/";

    // OneShot.setString(logRootSwerve + ".type", "SwerveDrive");
    OneShot.setString(logRootPose + ".type", "Field2d");

    RootLogging.getInstance()
        .addPoseLogger(logRootPose + "Robot", logMode, () -> drivetrain.getState().Pose)
        .addSwerveStateLogger(logRootSwerve, logMode, () -> drivetrain.getState().ModuleStates);
  }

  @Override
  public LogMode getLogMode() {
    return logMode;
  }
}
