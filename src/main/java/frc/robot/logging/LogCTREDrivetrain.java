package frc.robot.logging;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

public class LogCTREDrivetrain implements CompoundLogger {

  private final SwerveDrivetrain<?, ?, ?> drivetrain;
  private final LogMode logMode;
  private final String logRoot;

  private final String logRootSwerve;
  private final String logRootPose;

  public LogCTREDrivetrain(String logRoot, LogMode logMode, SwerveDrivetrain<?, ?, ?> drivetrain) {
    this.drivetrain = drivetrain;
    this.logMode = logMode;
    this.logRoot = logRoot;
    this.logRootSwerve = logRoot + "/Swerve/";
    this.logRootPose = logRoot + "/Pose/";

    OneShot.setString(logRootSwerve + ".type", "SwerveDrive");
    OneShot.setString(logRootPose + ".type", "Field2d");

    RootLogging.getInstance()
        .addPoseLogger(logRootPose, logMode, () -> drivetrain.getState().Pose)
        .addSwerveStateLogger(logRootSwerve, logMode, () -> drivetrain.getState().ModuleStates);
  }

  @Override
  public Object getOperatingObject() {
    return drivetrain;
  }

  @Override
  public LogMode getLogMode() {
    return logMode;
  }
}
