// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.sbdc.loggerhead.logging.LogMode;
import com.sbdc.loggerhead.logging.Loggerhead;
import com.sbdc.loggerhead.logging.compoundlogger.LogCTREDrivetrain;
import com.sbdc.loggerhead.logging.compoundlogger.LogPowerDistribution;
import com.sbdc.loggerhead.logging.compoundlogger.LogSubsystemCommands;
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
import frc.robot.commands.drivetrain.DrivePointAtAngle;
import frc.robot.commands.drivetrain.DriveTeleop;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.generated.TunerConstantsFake0621;
import frc.robot.sotm.SOTMState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.utils.DashboardStaticShotMap;
import frc.robot.utils.DynamicShotMap;
import frc.robot.utils.ShotMap;
import frc.robot.vision.MainVision;
import java.util.HashMap;

public final class RobotContainer {
  public static enum ArkelonActions {
    shootCommand,
    intakeHighCommand,
    intakeMidLowToggleCommand,
    intakeRunCommand,
    resetHeadingCommand;
  }

  private static final RobotContainer INSTANCE = new RobotContainer();

  public static RobotContainer getInstance() {
    return INSTANCE;
  }

  // Subsystems (and everything else)
  private final Drivetrain drivetrain = TunerConstantsFake0621.createDrivetrain();

  private final ShotMap dashShotMap = new DashboardStaticShotMap();
  private final SOTMState sotmstate = new SOTMState(drivetrain);
  private final DynamicShotMap dynamicShotMap = new DynamicShotMap(sotmstate);

  public final Flywheels flywheels = new Flywheels();
  private final Hood hood = new Hood();

  private final Indexer indexer = new Indexer();

  private final IntakePivot intakePivot = new IntakePivot();
  private final IntakeRoller intakeRoller = new IntakeRoller();

  private final HPoseEstimator hPoseEstimator = new HPoseEstimator(drivetrain);

  // Misc
  public final MainVision v =
      new MainVision(
          (Pose2d visionRobotPoseMeters,
              double timestampSeconds,
              Matrix<N3, N1> visionMeasurementStdDevs) -> {
            hPoseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
          },
          Robot.isReal()
              ? () -> Pose2d.kZero
              : () ->
                  drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose(),
          drivetrain);

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController coDriverController = new CommandXboxController(1);
  private final HashMap<String, Command> arkelonCommandMap = new HashMap<>();

  private final PowerDistribution pdh = new PowerDistribution(14, ModuleType.kRev);

  public RobotContainer() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    createActionMap();

    // DO replace these function calls with the new ones you currently want to use then redeploy
    // code
    configureCoDriverBindings();
    configureDriverBindings();

    SmartDashboard.putData(getAction(ArkelonActions.resetHeadingCommand));

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
        .addCompoundLogger(new LogCTREDrivetrain("LogCTREBultin", mainLogMode, drivetrain))
        .getSubTable("HPose")
        .addLoggable(hPoseEstimator, mainLogMode)
        .getParent()
        .getSubTable("drivetrainCustom")
        .addLoggable(drivetrain, mainLogMode);

    if (!Robot.isReal()) {
      subsystemTable
          .getSubTable("Drivetrain")
          .addStructLogger(
              "simTruePose",
              mainLogMode,
              () -> drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose(),
              Pose2d.struct);
    }

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

    rootTable.addStringLogger(
        "oops", mainLogMode, MatchContext.getInstance().getHubPose()::toString);

    rootTable.addDoubleLogger(
        "robotHubDistance",
        mainLogMode,
        () ->
            drivetrain
                .getState()
                .Pose
                .getTranslation()
                .getDistance(MatchContext.getInstance().getHubTranslation()));
  }

  private void createActionMap() {
    Command shootCommand =
        new ProxyCommand(new Shoot(flywheels, hood, indexer, dynamicShotMap))
            .alongWith(
                new ProxyCommand(
                    new DrivePointAtAngle(
                        drivetrain,
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightTriggerAxis,
                        sotmstate::getHeading)));

    Command resetHeadingCommand =
        Commands.runOnce(
            () -> {
              drivetrain.resetPose(new Pose2d(1, 1, Rotation2d.kZero));
              hPoseEstimator.reset(new Pose2d(1, 1, Rotation2d.kZero), true, true);
              // drivetrain.getPigeon2().setYaw(Degrees.of(0));
              // drivetrain.getPigeon2().reset();
              v.p2vwrapper.zero();

              if (!Robot.isReal()) {
                drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(
                    new Pose2d(1, 1, Rotation2d.kZero));
              }
            });

    Command intakeHighCommand = IntakeCommands.pivotHigh(intakePivot);
    Command intakeMidLowToggleCommand = IntakeCommands.pivotMidLowToggle(intakePivot);
    Command intakeRunCommand = IntakeCommands.rollerForward(intakeRoller);

    arkelonCommandMap.put("shootCommand", shootCommand);
    arkelonCommandMap.put("intakeHighCommand", intakeHighCommand);
    arkelonCommandMap.put("intakeMidLowToggleCommand", intakeMidLowToggleCommand);
    arkelonCommandMap.put("intakeRunCommand", intakeRunCommand);
    arkelonCommandMap.put("resetHeadingCommand", resetHeadingCommand);
  }

  private Command getAction(ArkelonActions action) {
    return arkelonCommandMap.get(action.toString());
  }

  // The configureDriver and configureCoDriver shoube duplicated
  // every time someone wants to use a different control scheme.
  // Do not modify the base functions
  // The new functions shouldbe called `configure<name><CoDriver/Driver>Binding`
  public void configureDriverBindings() {
    driverController.a().whileTrue(getAction(ArkelonActions.shootCommand));

    driverController.b().whileTrue(getAction(ArkelonActions.shootCommand));

    driverController.leftTrigger().whileTrue(getAction(ArkelonActions.intakeHighCommand));
    driverController.rightBumper().whileTrue(getAction(ArkelonActions.intakeMidLowToggleCommand));
    driverController.leftBumper().whileTrue(getAction(ArkelonActions.intakeRunCommand));

    driverController.povDown().onTrue(getAction(ArkelonActions.resetHeadingCommand));

    drivetrain.setDefaultCommand(
        new DriveTeleop(
            drivetrain,
            driverController::getLeftY,
            driverController::getLeftX,
            () -> -driverController.getRightX(),
            driverController::getRightTriggerAxis));
  }

  public void configureCoDriverBindings() {
    coDriverController.a().whileTrue(getAction(ArkelonActions.shootCommand));

    coDriverController.b().whileTrue(getAction(ArkelonActions.shootCommand));

    coDriverController.leftTrigger().whileTrue(getAction(ArkelonActions.intakeHighCommand));
    coDriverController.rightBumper().whileTrue(getAction(ArkelonActions.intakeMidLowToggleCommand));
    coDriverController.leftBumper().whileTrue(getAction(ArkelonActions.intakeRunCommand));

    coDriverController.povDown().onTrue(getAction(ArkelonActions.resetHeadingCommand));
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
            new IndexerCommands.ReverseIndexer(indexer).withTimeout(2),
            ShooterCommands.shooterDynamic(hood, flywheels, dynamicShotMap).withTimeout(3))
        .withTimeout(5);
  }
}
