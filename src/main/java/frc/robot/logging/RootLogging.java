package frc.robot.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
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

  private final List<SourceUpdateMap<PrimaryStringLog, String>> stringLogs = new ArrayList<>();
  private final List<SourceUpdateMap<PrimaryDoubleLog, Double>> doubleLogs = new ArrayList<>();
  private final List<SourceUpdateMap<PrimaryIntegerLog, Integer>> integerLogs = new ArrayList<>();
  private final List<SourceUpdateMap<PrimaryBooleanLog, Boolean>> booleanLogs = new ArrayList<>();

  private final HashMap<String, CompoundLogger> compoundLoggers = new HashMap<>();

  private boolean isDevMode;

  /**
   * Initializes logging framework by starting DataLogManager, disabling default logging of
   * NetworkTables, and hooking up the driver station logging
   */
  public void initializeLogging() {
    DataLogManager.start();
    LiveWindow.disableAllTelemetry();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(false);
  }

  public void setDevMode(boolean newDevMode) {
    isDevMode = newDevMode;
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

  public RootLogging addCompoundLogger(CompoundLogger compoundLogger) {
    compoundLoggers.put(compoundLogger.getName(), compoundLogger);
    return this;
  }

  public RootLogging addCompoundLogger(String name, CompoundLogger compoundLogger) {
    compoundLoggers.put(name, compoundLogger);
    return this;
  }

  public void update() {
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
