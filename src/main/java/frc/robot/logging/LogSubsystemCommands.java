package frc.robot.logging;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class LogSubsystemCommands implements CompoundLogger {
  public static final String subsystemTableRoot = "Subsystems";

  private final LogMode logMode;
  private final String name;
  private final String thisSubsystemRoot;

  private final Subsystem subsystem;

  public LogSubsystemCommands(LogMode logMode, Subsystem subsystem) {
    this.subsystem = subsystem;
    this.logMode = logMode;
    this.name = subsystem.getName();
    this.thisSubsystemRoot = subsystemTableRoot + "/" + name + "/" + name + "Commands/";

    // Publish only to network tables that it is of type subsystem (for display in dashboards)
    OneShot.setString(thisSubsystemRoot + ".type", "Subsystem");

    RootLogging.getInstance()
        .addBooleanLogger(
            thisSubsystemRoot + ".hasDefault", logMode, () -> subsystem.getDefaultCommand() != null)
        .addStringLogger(
            thisSubsystemRoot + ".default",
            logMode,
            () ->
                subsystem.getDefaultCommand() != null
                    ? subsystem.getDefaultCommand().getName()
                    : "none")
        .addBooleanLogger(
            thisSubsystemRoot + ".hasCommand", logMode, () -> subsystem.getCurrentCommand() != null)
        .addStringLogger(
            thisSubsystemRoot + ".command",
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
