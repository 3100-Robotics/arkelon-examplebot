package frc.robot.logging;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class LogSubsystemCommands implements CompoundLogger {
  public static final String subsystemTableRoot = "Subsystems";

  private final LogMode logMode;
  private final String name;
  private final String logRoot;

  private final Subsystem subsystem;

  /**
   * @param logRoot The path that the subsystems commands will get logged under. Does not end or
   *     begin with a slash.
   * @param logMode
   * @param subsystem
   */
  public LogSubsystemCommands(String logRoot, LogMode logMode, Subsystem subsystem) {
    this.subsystem = subsystem;
    this.logMode = logMode;
    this.name = subsystem.getName();
    // this.thisSubsystemRoot = subsystemTableRoot + "/" + name + "/" + name + "Commands/";
    this.logRoot = logRoot + "/" + name + "/Commands/";

    // Publish only to network tables that it is of type subsystem (for display in dashboards)
    OneShot.setString(this.logRoot + ".type", "Subsystem");

    RootLogging.getInstance()
        .addBooleanLogger(
            this.logRoot + ".hasDefault", logMode, () -> subsystem.getDefaultCommand() != null)
        .addStringLogger(
            this.logRoot + ".default",
            logMode,
            () ->
                subsystem.getDefaultCommand() != null
                    ? subsystem.getDefaultCommand().getName()
                    : "none")
        .addBooleanLogger(
            this.logRoot + ".hasCommand", logMode, () -> subsystem.getCurrentCommand() != null)
        .addStringLogger(
            this.logRoot + ".command",
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

  @Override
  public Object getOperatingObject() {
    return subsystem;
  }
}
