package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ShotMap;
import frc.robot.Targets.IndexerTarget;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;

public class Shoot extends ParallelCommandGroup {
  public Shoot(Flywheels flywheels, Hood hood, Indexer indexer, ShotMap shotMap) {
    addCommands(
        ShooterCommands.shooterDynamic(hood, flywheels, shotMap),
        new WaitCommand(1.1)
            .andThen(
                Commands.runEnd(
                    () -> indexer.setState(IndexerTarget.Forward),
                    () -> indexer.setState(IndexerTarget.Off),
                    indexer)));
  }
}
