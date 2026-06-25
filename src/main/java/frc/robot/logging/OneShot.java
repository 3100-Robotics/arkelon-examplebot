package frc.robot.logging;

import edu.wpi.first.networktables.NetworkTableInstance;

public class OneShot {
  public static void setString(String key, String value) {
    var publish = NetworkTableInstance.getDefault().getStringTopic(key).publish();
    publish.set(value);
  }

  public static void setDouble(String key, Double value) {
    var publish = NetworkTableInstance.getDefault().getDoubleTopic(key).publish();
    publish.set(value);
  }

  public static void setInteger(String key, Integer value) {
    var publish = NetworkTableInstance.getDefault().getIntegerTopic(key).publish();
    publish.set(value);
  }

  public static void setBoolean(String key, Boolean value) {
    var publish = NetworkTableInstance.getDefault().getBooleanTopic(key).publish();
    publish.set(value);
  }
}
