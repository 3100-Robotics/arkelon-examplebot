package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targets.IndexerTarget;
import frc.robot.subsystems.Indexer;

public class IndexerCommands {
  public static class ReverseIndexer extends Command {
    private final Indexer indexer;

    public ReverseIndexer(Indexer indexer) {
      this.indexer = indexer;
      addRequirements(indexer);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      indexer.setState(IndexerTarget.Reverse);
    }

    @Override
    public void end(boolean interrupted) {
      indexer.setState(IndexerTarget.Off);
    }
  }
}
