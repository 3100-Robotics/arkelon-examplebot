package frc.robot.logging.compoundlogger;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.logging.LogMode;
import frc.robot.logging.OneShot;
import frc.robot.logging.RootLogging;

public class LogSubsystemCommands implements CompoundLogger {
  public static final String subsystemTableRoot = "Subsystems";

  private final LogMode logMode;
  private final String name;

  private final Subsystem subsystem;

  /**
   * Log the current command the subsystem is running, default command
   *
   * @param name The path that the subsystems commands will get logged under. Does not end or begin
   *     with a slash.
   * @param logMode
   * @param subsystem
   */
  public LogSubsystemCommands(String name, LogMode logMode, Subsystem subsystem) {
    this.subsystem = subsystem;
    this.logMode = logMode;
    this.name = name;
  }

  @Override
  public void initialize(String parentTable) {
    String logRoot = parentTable + name + "/";

    // Publish only to network tables that it is of type subsystem (for display in dashboards)
    OneShot.setString(logRoot + ".type", "Subsystem");

    RootLogging.getInstance()
        .addBooleanLogger(
            logRoot + ".hasDefault", logMode, () -> subsystem.getDefaultCommand() != null)
        .addStringLogger(
            logRoot + ".default",
            logMode,
            () ->
                subsystem.getDefaultCommand() != null
                    ? subsystem.getDefaultCommand().getName()
                    : "none")
        .addBooleanLogger(
            logRoot + ".hasCommand", logMode, () -> subsystem.getCurrentCommand() != null)
        .addStringLogger(
            logRoot + ".command",
            logMode,
            () ->
                subsystem.getCurrentCommand() != null
                    ? subsystem.getCurrentCommand().getName()
                    : "none");
  }

  @Override
  public void update() {}

  @Override
  public LogMode getLogMode() {
    return this.logMode;
  }
}
