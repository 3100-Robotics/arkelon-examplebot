// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstantsFake0621;
import frc.robot.logging.LogMode;
import frc.robot.logging.LogSubsystem;
import frc.robot.logging.LogXboxController;
import frc.robot.logging.RootLogging;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;

public class RobotContainer {
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

    // Without this line no logging will happen
    RootLogging.getInstance().initializeLogging();
    RootLogging.getInstance()
        .addCompoundLogger(new LogXboxController("evenCtl", evenController))
        .addBooleanLogger("testKey", LogMode.Both, () -> false)
        .addCompoundLogger(new LogSubsystem(LogMode.Both, drivetrain))
        .addCompoundLogger(new LogSubsystem(LogMode.Both, flywheels))
        .addCompoundLogger(new LogSubsystem(LogMode.Both, hood))
        .addCompoundLogger(new LogSubsystem(LogMode.Both, indexer))
        .addCompoundLogger(new LogSubsystem(LogMode.Both, intakePivot))
        .addCompoundLogger(new LogSubsystem(LogMode.Both, intakeRoller));
  }

  public void registerPeriodics(Robot robot) {
    // update is a lambda method
    robot.addPeriodic(RootLogging.getInstance()::update, 0.02);
  }

  private void configureBindings() {
    evenController.a().whileTrue(new Shoot(flywheels, hood, indexer, shotMap));

    drivetrain.setDefaultCommand(
        new DriveTeleop(
            drivetrain,
            evenController::getLeftX,
            evenController::getLeftY,
            evenController::getRightX,
            evenController::getRightTriggerAxis));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
