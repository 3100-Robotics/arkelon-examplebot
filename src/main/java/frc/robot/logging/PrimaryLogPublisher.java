package frc.robot.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import java.util.function.Consumer;

public abstract class PrimaryLogPublisher<T, DLE extends DataLogEntry, P extends Publisher> {
  private final LogMode mode;
  private final String name;

  private final NetworkTableInstance ntInstance;
  private final DataLog dataLog;

  private DLE logEntry;
  private P ntPublish;

  private Consumer<T> updateIndirection;

  private T last;

  public PrimaryLogPublisher(
      String key, LogMode mode, NetworkTableInstance ntInstance, DataLog dataLog) {
    this.name = getClass() + " " + key;
    this.mode = mode;
    this.ntInstance = ntInstance;
    this.dataLog = dataLog;

    switch (mode) {
      case FileOnly:
        updateIndirection = newValue -> updateFile(newValue);
        break;
      case NetworkOnly:
        updateIndirection = newValue -> updateNetwork(newValue);
        break;
      case Both:
        updateIndirection =
            newValue -> {
              updateFile(newValue);
              updateNetwork(newValue);
            };
        break;
    }
  }

  public String getName() {
    return name;
  }

  public LogMode getMode() {
    return mode;
  }

  public void update(T newValue) {
    if (last != newValue) {
      updateIndirection.accept(newValue);

      // switch (mode) {
      //   case FileOnly:
      //     updateFile(newValue);
      //     break;
      //   case NetworkOnly:
      //     updateNetwork(newValue);
      //     break;
      //   case Both:
      //     updateFile(newValue);
      //     updateNetwork(newValue);
      //     break;
      // }
    }
  }

  protected void setLogEntry(DLE logEntry) {
    this.logEntry = logEntry;
  }

  protected void setPublisher(P publisher) {
    this.ntPublish = publisher;
  }

  protected DLE getLogEntry() {
    return logEntry;
  }

  protected P getPublisher() {
    return ntPublish;
  }

  protected DataLog getDataLog() {
    return dataLog;
  }

  protected NetworkTableInstance getNtInstance() {
    return ntInstance;
  }

  protected abstract void updateFile(T newValue);

  protected abstract void updateNetwork(T newValue);
}
