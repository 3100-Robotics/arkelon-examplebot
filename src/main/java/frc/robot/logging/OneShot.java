package frc.robot.logging;

import edu.wpi.first.networktables.NetworkTableInstance;

/** Publish values to NetworkTables one time */
public class OneShot {
  /**
   * Publishes string value to NetworkTables. Do not call repeatedly.
   *
   * @param key NetworkTables key
   * @param value The string
   */
  public static void setString(String key, String value) {
    var publish = NetworkTableInstance.getDefault().getStringTopic(key).publish();
    publish.set(value);
  }

  /**
   * Publishes double value to NetworkTables. Do not call repeatedly.
   *
   * @param key NetworkTables key
   * @param value The double
   */
  public static void setDouble(String key, Double value) {
    var publish = NetworkTableInstance.getDefault().getDoubleTopic(key).publish();
    publish.set(value);
  }

  /**
   * Publishes integer value to NetworkTables. Do not call repeatedly.
   *
   * @param key NetworkTables key
   * @param value The integer
   */
  public static void setInteger(String key, Integer value) {
    var publish = NetworkTableInstance.getDefault().getIntegerTopic(key).publish();
    publish.set(value);
  }

  /**
   * Publishes boolean value to NetworkTables. Do not call repeatedly.
   *
   * @param key NetworkTables key
   * @param value The boolean
   */
  public static void setBoolean(String key, Boolean value) {
    var publish = NetworkTableInstance.getDefault().getBooleanTopic(key).publish();
    publish.set(value);
  }
}
