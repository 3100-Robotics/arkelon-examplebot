package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targets.IntakePivotTarget;
import frc.robot.subsystems.IntakePivot;

public class IntakePivotHigh extends Command {
  private IntakePivotTarget stateBeforeHigh;
  private IntakePivot intakePivot;

  public IntakePivotHigh(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
  }

  @Override
  public void initialize() {
    stateBeforeHigh = intakePivot.getState();
  }

  @Override
  public void execute() {
    intakePivot.setState(IntakePivotTarget.High);
  }

  @Override
  public void end(boolean inturupted) {
    intakePivot.setState(stateBeforeHigh);
  }
}
