// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.sbdc.loggerhead.LogMode;
import com.sbdc.loggerhead.Loggerhead;
import com.sbdc.loggerhead.compoundlogger.LogCTREDrivetrain;
import com.sbdc.loggerhead.compoundlogger.LogPowerDistribution;
import com.sbdc.loggerhead.compoundlogger.LogSubsystemCommands;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IndexerCommands;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.drivetrain.DrivePointAtPose;
import frc.robot.commands.drivetrain.DriveTeleop;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.generated.TunerConstantsFake0621;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.utils.DashboardStaticShotMap;
import frc.robot.utils.ShotMap;
import frc.robot.vision.MainVision;

public final class RobotContainer {
  private static final RobotContainer INSTANCE = new RobotContainer();

  public static RobotContainer getInstance() {
    return INSTANCE;
  }

  private final ShotMap shotMap = new DashboardStaticShotMap();

  // Subsystems
  private final Drivetrain drivetrain = TunerConstantsFake0621.createDrivetrain();

  public final Flywheels flywheels = new Flywheels();
  private final Hood hood = new Hood();

  private final Indexer indexer = new Indexer();

  private final IntakePivot intakePivot = new IntakePivot();
  private final IntakeRoller intakeRoller = new IntakeRoller();

  private final HPoseEstimator hPoseEstimator = new HPoseEstimator(drivetrain);

  private final MatchContext matchContext = new MatchContext();

  // Misc
  public final MainVision v =
      new MainVision(
          (Pose2d visionRobotPoseMeters,
              double timestampSeconds,
              Matrix<N3, N1> visionMeasurementStdDevs) -> {
            hPoseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
          },
          drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive::getSimulatedDriveTrainPose,
          drivetrain);

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
    visionTable.addLoggable(v, mainLogMode);

    var subsystemTable = rootTable.getSubTable("Subsystems");
    subsystemTable
        .getSubTable("Drivetrain")
        .addCompoundLogger(new LogSubsystemCommands("Commands", mainLogMode, drivetrain))
        .addCompoundLogger(new LogCTREDrivetrain("Swerve", mainLogMode, drivetrain))
        .addPoseLogger(
            "simTruePose",
            mainLogMode,
            () -> drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose())
        .addLoggable(hPoseEstimator, mainLogMode)
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

    rootTable
        .getSubTable("SystemInfo")
        .addDoubleLogger("looptime", LogMode.NetworkOnly, () -> Robot.getInstance().looptime / 1000)
        .addIntegerLogger("overruns", LogMode.NetworkOnly, () -> Robot.getInstance().overruns)
        .addDoubleLogger(
            "timeSinceLastOverrun",
            LogMode.NetworkOnly,
            () -> Robot.getInstance().ilooptime / 1000000);
  }

  private void configureBindings() {
    evenController
        .a()
        .whileTrue(
            new ProxyCommand(new Shoot(flywheels, hood, indexer, shotMap))
                .alongWith(
                    new ProxyCommand(
                        new DrivePointAtPose(
                            drivetrain,
                            evenController::getLeftY,
                            evenController::getLeftX,
                            evenController::getRightTriggerAxis,
                            matchContext::getHubPose))));
    evenController.b().onTrue(Commands.runOnce(() -> Loggerhead.getInstance().cleanLoggers()));
    evenController.x().onTrue(Commands.runOnce(() -> configureLogging()));
    drivetrain.setDefaultCommand(
        new DriveTeleop(
            drivetrain,
            evenController::getLeftY,
            evenController::getLeftX,
            evenController::getRightX,
            evenController::getRightTriggerAxis));

    evenController.leftTrigger().whileTrue(IntakeCommands.pivotHigh(intakePivot));
    evenController.leftBumper().whileTrue(IntakeCommands.pivotMidLowToggle(intakePivot));
    evenController.rightBumper().whileTrue(IntakeCommands.rollerForward(intakeRoller));

    SmartDashboard.putData(
        Commands.runOnce(
            () -> {
              drivetrain.resetPose(new Pose2d(1, 1, Rotation2d.kZero));
              drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(
                  new Pose2d(1, 1, Rotation2d.kZero));
              hPoseEstimator.reset(new Pose2d(1, 1, Rotation2d.kZero), true, true);
            }));
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return Commands.sequence(
            new IndexerCommands.ReverseIndexer(indexer).withTimeout(2),
            ShooterCommands.shooterDynamic(hood, flywheels, shotMap).withTimeout(3))
        .withTimeout(5);
  }
}
