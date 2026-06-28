package frc.robot.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.logging.configuration.Configurator;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class RootLogging {
  public static class LoggerSetDescriptor {}

  public static class SourceUpdateMap<T extends PrimaryLogPublisher<E, ?, ?>, E> {
    public final T log;
    public final Supplier<E> newValue;
    public final RootLogging root;
    public LogMode devLogMode;

    public SourceUpdateMap(RootLogging root, T log, Supplier<E> newValue) {
      this.root = root;
      this.log = log;
      this.newValue = newValue;
    }
  }

  private static final RootLogging INSTANCE = new RootLogging();

  public static RootLogging getInstance() {
    return INSTANCE;
  }

  private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  private final DataLog log = DataLogManager.getLog();
  private final Configurator configurator = new Configurator(this::cleanLoggers, () -> {});

  private final List<SourceUpdateMap<PrimaryStringLog, String>> stringLogs = new ArrayList<>();
  private final List<SourceUpdateMap<PrimaryDoubleLog, Double>> doubleLogs = new ArrayList<>();
  private final List<SourceUpdateMap<PrimaryIntegerLog, Integer>> integerLogs = new ArrayList<>();
  private final List<SourceUpdateMap<PrimaryBooleanLog, Boolean>> booleanLogs = new ArrayList<>();
  private final List<SourceUpdateMap<PrimaryPoseLog, Pose2d>> poseLogs = new ArrayList<>();
  private final List<SourceUpdateMap<PrimarySwerveStateLog, SwerveModuleState[]>> swerveStateLogs =
      new ArrayList<>();

  private final HashMap<Object, CompoundLogger> compoundLoggers = new HashMap<>();

  private RootLogging() {}

  /**
   * Initializes logging framework by starting DataLogManager, disabling default logging of
   * NetworkTables, and hooking up the driver station logging
   */
  public void initializeLogging() {
    DataLogManager.start();
    LiveWindow.disableAllTelemetry();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(false);

    configurator.getConfiguratorCallback().run();
  }

  public RootLogging addStringLogger(String key, LogMode mode, Supplier<String> stringGetter) {
    PrimaryStringLog logPub = new PrimaryStringLog(key, mode, ntInst, log);
    SourceUpdateMap<PrimaryStringLog, String> compundLogger =
        new SourceUpdateMap<>(this, logPub, stringGetter);
    stringLogs.add(compundLogger);

    return this;
  }

  public RootLogging addDoubleLogger(String key, LogMode mode, Supplier<Double> doubleGetter) {
    PrimaryDoubleLog logPub = new PrimaryDoubleLog(key, mode, ntInst, log);
    SourceUpdateMap<PrimaryDoubleLog, Double> compundLogger =
        new SourceUpdateMap<>(this, logPub, doubleGetter);
    doubleLogs.add(compundLogger);

    return this;
  }

  public RootLogging addIntegerLogger(String key, LogMode mode, Supplier<Integer> doubleGetter) {
    PrimaryIntegerLog logPub = new PrimaryIntegerLog(key, mode, ntInst, log);
    SourceUpdateMap<PrimaryIntegerLog, Integer> compundLogger =
        new SourceUpdateMap<>(this, logPub, doubleGetter);
    integerLogs.add(compundLogger);

    return this;
  }

  public RootLogging addBooleanLogger(String key, LogMode mode, Supplier<Boolean> boolGetter) {
    PrimaryBooleanLog logPub = new PrimaryBooleanLog(key, mode, ntInst, log);
    SourceUpdateMap<PrimaryBooleanLog, Boolean> compundLogger =
        new SourceUpdateMap<>(this, logPub, boolGetter);
    booleanLogs.add(compundLogger);

    return this;
  }

  public RootLogging addPoseLogger(String key, LogMode mode, Supplier<Pose2d> poseGetter) {
    PrimaryPoseLog logPub = new PrimaryPoseLog(key, mode, ntInst, log);
    SourceUpdateMap<PrimaryPoseLog, Pose2d> compundLogger =
        new SourceUpdateMap<>(this, logPub, poseGetter);
    poseLogs.add(compundLogger);

    return this;
  }

  public RootLogging addSwerveStateLogger(
      String key, LogMode mode, Supplier<SwerveModuleState[]> moduleStateGetter) {
    PrimarySwerveStateLog logPub = new PrimarySwerveStateLog(key, mode, ntInst, log);
    SourceUpdateMap<PrimarySwerveStateLog, SwerveModuleState[]> compundLogger =
        new SourceUpdateMap<>(this, logPub, moduleStateGetter);
    swerveStateLogs.add(compundLogger);

    return this;
  }

  public RootLogging addCompoundLogger(CompoundLogger compoundLogger) {
    compoundLoggers.put(compoundLogger.getName(), compoundLogger);
    return this;
  }

  public RootLogging addCompoundLogger(String key, CompoundLogger compoundLogger) {
    compoundLoggers.put(key, compoundLogger);
    return this;
  }

  // Special Compounds
  public RootLogging addSubsystemCommandLogger(String logRoot, LogMode mode, Subsystem subsystem) {
    addCompoundLogger(new LogSubsystemCommands(logRoot, mode, subsystem));
    return this;
  }

  public RootLogging addXboxControllerLogger(String name, CommandXboxController controller) {
    addCompoundLogger(name, new LogXboxController(name, controller));
    return this;
  }

  // Configuration
  public RootLogging applyToConfigurator(Consumer<Configurator> applyTo) {
    applyTo.accept(configurator);
    return this;
  }

  public void cleanLoggers() {
    stringLogs.clear();
    doubleLogs.clear();
    integerLogs.clear();
    booleanLogs.clear();
    poseLogs.clear();
    swerveStateLogs.clear();
    compoundLoggers.clear();

    // ntInst.stopLocal();
    // ntInst.startServer();
    // TODO figure out how to change logs
    // DataLogManager.stop();
    // DataLogManager.start(null, null);
  }

  public void update() {
    if (DriverStation.isDisabled()) {
      configurator.checkAllHooks();
    }

    for (SourceUpdateMap<PrimaryStringLog, String> srcUpdateMap : stringLogs) {
      srcUpdateMap.log.update(srcUpdateMap.newValue.get());
    }

    for (SourceUpdateMap<PrimaryDoubleLog, Double> srcUpdateMap : doubleLogs) {
      srcUpdateMap.log.update(srcUpdateMap.newValue.get());
    }

    for (SourceUpdateMap<PrimaryIntegerLog, Integer> srcUpdateMap : integerLogs) {
      srcUpdateMap.log.update(srcUpdateMap.newValue.get());
    }

    for (SourceUpdateMap<PrimaryBooleanLog, Boolean> srcUpdateMap : booleanLogs) {
      srcUpdateMap.log.update(srcUpdateMap.newValue.get());
    }

    for (SourceUpdateMap<PrimaryPoseLog, Pose2d> srcUpdateMap : poseLogs) {
      srcUpdateMap.log.update(srcUpdateMap.newValue.get());
    }

    for (SourceUpdateMap<PrimarySwerveStateLog, SwerveModuleState[]> srcUpdateMap :
        swerveStateLogs) {
      srcUpdateMap.log.update(srcUpdateMap.newValue.get());
    }

    for (CompoundLogger compoundLogger : compoundLoggers.values()) {
      compoundLogger.update();
    }
  }

  public DataLog getDataLog() {
    return log;
  }

  public NetworkTableInstance getNetworkTableInstance() {
    return ntInst;
  }
}
