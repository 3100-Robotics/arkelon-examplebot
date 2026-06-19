package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ShotMap;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Hood;

/** Container class for all commands that operate on the structure */
public class ShooterCommands {
  /**
   * Runs the the {@link Flywheels} to the target angular velocity provided by the {@link ShotMap}
   *
   * @param flywheels the flywheels sybsystem
   * @param shotMap the shotmap to run the flywheels to
   * @return the command
   */
  public static Command flywheelsDynamic(Flywheels flywheels, ShotMap shotMap) {
    return Commands.runEnd(
            () -> {
              flywheels.setSpeed(shotMap.getFlywheelSpeed());
            },
            () -> {
              flywheels.stop();
            },
            flywheels)
        .withName("flywheelsDynamic w target: " + shotMap.getTarget());
  }

  /**
   * Runs the the {@link Hood} to the target angle provided by the {@link ShotMap}
   *
   * @param hood the hood sybsystem
   * @param shotMap the shotmap to run the hood to
   * @return the command
   */
  public static Command hoodDynamic(Hood hood, ShotMap shotMap) {
    return Commands.runEnd(
            () -> {
              hood.setHoodAngle(shotMap.getHoodAngle());
            },
            () -> {
              hood.stopHood();
            },
            hood)
        .withName("hoodDynamic w target: " + shotMap.getTarget());
  }

  /**
   * Runs the the {@link Hood} and the {@link Flywheels} to the targets provided by the {@link
   * ShotMap}
   *
   * @param hood the hood sybsystem
   * @param flywheels the flywheels sybsystem
   * @param shotMap the shotmap to run the Hood and Flywheels to
   * @return the command
   */
  public static Command shooterDynamic(Hood hood, Flywheels flywheels, ShotMap shotMap) {
    return hoodDynamic(hood, shotMap)
        .alongWith(flywheelsDynamic(flywheels, shotMap))
        .withName("shooterDynamic w target: " + shotMap.getTarget());
  }
}
