// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.compoundlogger.LogCTREDrivetrain;
import com.sbdc.loggerhead.compoundlogger.LogPowerDistribution;
import com.sbdc.loggerhead.compoundlogger.LogSubsystemCommands;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
import frc.robot.vision.MainVision;

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

  // Misc
  public final MainVision poseGetter =
      new MainVision((Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) -> {});

  public final CommandXboxController evenController = new CommandXboxController(0);

  private final PowerDistribution pdh = new PowerDistribution(14, ModuleType.kRev);

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
    LogMode mainLogMode = DriverStation.isFMSAttached() ? LogMode.FileOnly : LogMode.Both;
    mainLogMode = Robot.isReal() ? mainLogMode : LogMode.NetworkOnly;

    var rootTable = Loggerhead.getInstance().getRootTable();

    var visionTable = rootTable.getSubTable("Vision");
    visionTable.addLoggableUnder("PoseGetter", poseGetter, mainLogMode);

    var subsystemTable = rootTable.getSubTable("Subsystems");
    subsystemTable
        .getSubTable("Drivetrain")
        .addCompoundLogger(new LogSubsystemCommands("Commands", mainLogMode, drivetrain))
        .addCompoundLogger(new LogCTREDrivetrain("Swerve", mainLogMode, drivetrain))
        .addLoggable(drivetrain, mainLogMode);

    subsystemTable
        .getSubTable("Hood")
        .addCompoundLogger(new LogSubsystemCommands("Commands", mainLogMode, hood))
        .addLoggable(hood, mainLogMode);

    subsystemTable
        .getSubTable("Flywheels")
        .addCompoundLogger(new LogSubsystemCommands("Commands", mainLogMode, flywheels))
        .addLoggable(flywheels, mainLogMode);

    subsystemTable
        .getSubTable("Indexer")
        .addCompoundLogger(new LogSubsystemCommands("Commands", mainLogMode, indexer))
        .addLoggable(indexer, mainLogMode);

    subsystemTable
        .getSubTable("IntakePivot")
        .addCompoundLogger(new LogSubsystemCommands("Commands", mainLogMode, intakePivot))
        .addLoggable(intakePivot, mainLogMode);

    subsystemTable
        .getSubTable("IntakeRoller")
        .addCompoundLogger(new LogSubsystemCommands("Commands", mainLogMode, intakeRoller))
        .addLoggable(intakeRoller, mainLogMode);

    rootTable.getSubTable("PDH").addCompoundLogger(new LogPowerDistribution(mainLogMode, pdh));
  }

  private void configureBindings() {
    evenController.a().whileTrue(new Shoot(flywheels, hood, indexer, shotMap));
    evenController.b().onTrue(Commands.runOnce(() -> Loggerhead.getInstance().cleanLoggers()));
    evenController.x().onTrue(Commands.runOnce(() -> configureLogging()));
    drivetrain.setDefaultCommand(
        new DriveTeleop(
            drivetrain,
            evenController::getLeftY,
            evenController::getLeftX,
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
