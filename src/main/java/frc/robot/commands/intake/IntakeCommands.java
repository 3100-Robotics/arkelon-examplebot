package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Targets.IntakePivotTarget;
import frc.robot.subsystems.IntakePivot;

public interface IntakeCommands {
  public static Command pivotLow(IntakePivot intakePivot) {
    return Commands.run(() -> intakePivot.setState(IntakePivotTarget.Low), intakePivot)
        .withName("IntakePivotLow");
  }

  public static Command pivotMid(IntakePivot intakePivot) {
    return Commands.run(() -> intakePivot.setState(IntakePivotTarget.Medium), intakePivot)
        .withName("IntakePivotHigh");
  }

  public static Command pivotHigh(IntakePivot intakePivot) {
    return new IntakePivotHigh(intakePivot);
  }
}
