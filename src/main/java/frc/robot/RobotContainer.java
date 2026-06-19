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
import frc.robot.commands.Shoot;
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
  private final Flywheels flywheels = new Flywheels();
  private final Hood hood = new Hood();

  private final Indexer indexer = new Indexer();

  private final IntakePivot intakePivot = new IntakePivot();
  private final IntakeRoller intakeRoller = new IntakeRoller();

  public final CommandXboxController evenController = new CommandXboxController(0);

  public RobotContainer() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    Logging.getInstance();
    configureBindings();
  }

  private void configureBindings() {
    // evenController.a().whileTrue(Commands.print("A"));
    // evenController.b().whileTrue(Commands.print("B"));
    // evenController.x().whileTrue(Commands.print("X"));
    // evenController.y().whileTrue(Commands.print("Y"));
    // evenController.leftBumper().whileTrue(Commands.print("LB"));
    // evenController.rightBumper().whileTrue(Commands.print("RB"));
    // evenController.leftStick().whileTrue(Commands.print("LSTCK"));
    // evenController.rightStick().whileTrue(Commands.print("RSTCK"));
    evenController.a().whileTrue(new Shoot(flywheels, hood, indexer, shotMap));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
