// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.compoundlogger.LogCTREDrivetrain;
import com.sbdc.loggerhead.compoundlogger.LogSubsystemCommands;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstantsFake0621;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;

public final class RobotContainer {
  private static final RobotContainer INSTANCE = new RobotContainer();

  public static RobotContainer getInstance() {
    return INSTANCE;
  }

  private final ShotMap shotMap =
      new ShotMap() {
        {
          SmartDashboard.putNumber("Hood Angle", 31);
          SmartDashboard.putNumber("Flywheel Speed", 3100);
        }

        public String getTarget() {
          return "Hub";
        }

        public AngularVelocity getFlywheelSpeed() {
          return RPM.of(SmartDashboard.getNumber("Flywheel Speed", 5100));
        }

        public Angle getHoodAngle() {
          return Degrees.of(SmartDashboard.getNumber("Hood Angle", 51));
        }
      };

  // Subsystems
  private final Drivetrain drivetrain = TunerConstantsFake0621.createDrivetrain();

  public final Flywheels flywheels = new Flywheels();
  private final Hood hood = new Hood();

  private final Indexer indexer = new Indexer();

  private final IntakePivot intakePivot = new IntakePivot();
  private final IntakeRoller intakeRoller = new IntakeRoller();

  public final CommandXboxController evenController = new CommandXboxController(0);

  public RobotContainer() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    configureBindings();

    Loggerhead.getInstance()
        .applyToConfigurator(
            configurator ->
                configurator
                    .setConfigureCallback(this::configureLogging)
                    .addHook(DriverStation::isFMSAttached))
        .initializeLogging(); // Must be the final call in the configuration chain
  }

  public void registerPeriodics(Robot robot) {
    robot.addPeriodic(Loggerhead.getInstance()::update, 0.02);
  }

  private void configureLogging() {
    LogMode noNetOnField = DriverStation.isFMSAttached() ? LogMode.FileOnly : LogMode.Both;

    var rootTable = Loggerhead.getInstance().getRootTable();
    var subsystemTable = rootTable.getSubTable("Subsystems");
    subsystemTable
        .getSubTable("Drivetrain")
        .addCompoundLogger(new LogSubsystemCommands("Commands", noNetOnField, drivetrain))
        .addCompoundLogger(new LogCTREDrivetrain("Swerve", noNetOnField, drivetrain))
        .addLoggable(drivetrain, noNetOnField);

    subsystemTable
        .getSubTable("Hood")
        .addCompoundLogger(new LogSubsystemCommands("Commands", noNetOnField, hood))
        .addLoggable(hood, noNetOnField);

    subsystemTable
        .getSubTable("Flywheels")
        .addCompoundLogger(new LogSubsystemCommands("Commands", noNetOnField, flywheels))
        .addLoggable(flywheels, noNetOnField);

    subsystemTable
        .getSubTable("Indexer")
        .addCompoundLogger(new LogSubsystemCommands("Commands", noNetOnField, indexer))
        .addLoggable(indexer, noNetOnField);

    subsystemTable
        .getSubTable("IntakePivot")
        .addCompoundLogger(new LogSubsystemCommands("Commands", noNetOnField, intakePivot))
        .addLoggable(intakePivot, noNetOnField);

    subsystemTable
        .getSubTable("IntakeRoller")
        .addCompoundLogger(new LogSubsystemCommands("Commands", noNetOnField, intakeRoller))
        .addLoggable(intakeRoller, noNetOnField);
  }

  private void configureBindings() {
    evenController.a().whileTrue(new Shoot(flywheels, hood, indexer, shotMap));
    evenController.b().onTrue(Commands.runOnce(() -> Loggerhead.getInstance().cleanLoggers()));
    evenController.x().onTrue(Commands.runOnce(() -> configureLogging()));
    drivetrain.setDefaultCommand(
        new DriveTeleop(
            drivetrain,
            evenController::getLeftX,
            evenController::getLeftY,
            evenController::getRightX,
            evenController::getRightTriggerAxis));
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return Commands.sequence(
            new IndexerCommands.ReverseIndexer(indexer).withTimeout(2),
            ShooterCommands.shooterDynamic(hood, flywheels, shotMap).withTimeout(3))
        .withTimeout(5);
  }
}
