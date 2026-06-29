package frc.robot.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.logging.compoundlogger.CompoundLogger;
import frc.robot.logging.exceptions.LoggingTableRootDefinedError;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Table {
  private static Table ROOT;

  protected static void setRoot(Table table) {
    if (table != null) {
      ROOT = table;
    }
  }

  public static boolean hasRoot() {
    return ROOT != null;
  }

  public final String name;
  public final String path;

  private final Table parent;
  private final RootLogging rootLogging;
  private final HashMap<String, Table> children = new HashMap<>();

  public Table(String name, RootLogging rootLogging, Table parent) {
    this.parent = parent;
    this.rootLogging = rootLogging;
    this.name = name;
    this.path = parent.path + name + "/";
  }

  /**
   * Creates the root table
   *
   * @param name
   * @param rootLogging
   */
  protected Table(String name, RootLogging rootLogging) {
    if (hasRoot()) {
      throw new LoggingTableRootDefinedError();
    }
    this.parent = this;
    setRoot(this);
    this.rootLogging = rootLogging;
    this.name = name;
    this.path = "";
  }

  public Table applyToSubTable(String newTableName, Consumer<Table> applyToTable) {
    Table table = getSubTable(newTableName);
    applyToTable.accept(table);
    return this;
  }

  public Table getSubTable(String newTableName) {
    if (children.containsKey(newTableName)) {
      return children.get(newTableName);
    }

    Table newTable = new Table(newTableName, rootLogging, this);
    children.put(newTableName, newTable);
    return newTable;
  }

  public HashMap<String, Table> getChildren() {
    return children;
  }

  public Table getParent() {
    return parent;
  }

  public Table addStringLogger(String key, LogMode mode, Supplier<String> stringGetter) {
    rootLogging.addStringLogger(path + key, mode, stringGetter);
    return this;
  }

  public Table addDoubleLogger(String key, LogMode mode, Supplier<Double> doubleGetter) {
    rootLogging.addDoubleLogger(path + key, mode, doubleGetter);
    return this;
  }

  public Table addIntegerLogger(String key, LogMode mode, Supplier<Integer> doubleGetter) {
    rootLogging.addIntegerLogger(path + key, mode, doubleGetter);
    return this;
  }

  public Table addBooleanLogger(String key, LogMode mode, Supplier<Boolean> boolGetter) {
    rootLogging.addBooleanLogger(path + key, mode, boolGetter);
    return this;
  }

  public Table addPoseLogger(String key, LogMode mode, Supplier<Pose2d> poseGetter) {
    rootLogging.addPoseLogger(path + key, mode, poseGetter);
    return this;
  }

  public Table addSwerveStateLogger(
      String key, LogMode mode, Supplier<SwerveModuleState[]> moduleStateGetter) {
    rootLogging.addSwerveStateLogger(path + key, mode, moduleStateGetter);
    return this;
  }

  public Table addCompoundLogger(CompoundLogger compoundLogger) {
    rootLogging.addCompoundLogger(compoundLogger);
    compoundLogger.initialize(path);
    return this;
  }
}
