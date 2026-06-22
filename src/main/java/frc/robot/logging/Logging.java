package frc.robot.logging;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

public class Logging {
  private static final Logging INSTANCE = new Logging();

  public static Logging getInstance() {
    return INSTANCE;
  }

  private boolean publishTimings = false;
  public Runnable periodic = () -> periodicNoTiming();

  private final NetworkTableInstance NTInst = NetworkTableInstance.getDefault();
  private final NetworkTable timingPublisherTable = NTInst.getTable("timings");

  private List<Pair<RunsPeriodic, DoublePublisher>> logsPeriodicWithTiming = new ArrayList<>();

  private Logging() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);
  }

  public void setPublishTimings(boolean publishTimings) {
    this.publishTimings = publishTimings;
    if (publishTimings) {
      periodic = () -> periodicTiming();
    } else {
      periodic = () -> periodicNoTiming();
    }
  }

  public boolean getPublishTimings() {
    return publishTimings;
  }

  public void addLogger(RunsPeriodic logger) {
    logsPeriodicWithTiming.add(
        Pair.of(logger, timingPublisherTable.getDoubleTopic(logger.getName()).publish()));
  }

  private void periodicTiming() {
    for (Pair<RunsPeriodic, DoublePublisher> periodicAndPublisher : logsPeriodicWithTiming) {
      RunsPeriodic hasPeriodic = periodicAndPublisher.getFirst();
      DoublePublisher publisher = periodicAndPublisher.getSecond();
      double startTime = Timer.getFPGATimestamp();
      hasPeriodic.periodic();
      double endTime = Timer.getFPGATimestamp();
      publisher.set(endTime - startTime);
    }
  }

  private void periodicNoTiming() {
    for (Pair<RunsPeriodic, DoublePublisher> periodic : logsPeriodicWithTiming) {
      periodic.getFirst().periodic();
    }
  }
}
