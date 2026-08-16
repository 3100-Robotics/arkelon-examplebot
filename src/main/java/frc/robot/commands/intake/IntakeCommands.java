package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Targets.IntakePivotTarget;
import frc.robot.Targets.IntakeRollerTarget;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;

public interface IntakeCommands {
  public static Command pivotMidLowToggle(IntakePivot intakePivot) {
    return Commands.either(
        IntakeCommands.pivotLow(intakePivot),
        IntakeCommands.pivotMid(intakePivot),
        () -> intakePivot.getState() == IntakePivotTarget.Medium);
  }

  public static Command pivotLow(IntakePivot intakePivot) {
    return Commands.run(() -> intakePivot.setState(IntakePivotTarget.Low), intakePivot)
        .withName("IntakePivotLow");
  }

  public static Command pivotMid(IntakePivot intakePivot) {
    return Commands.run(() -> intakePivot.setState(IntakePivotTarget.Medium), intakePivot)
        .withName("IntakePivotMid");
  }

  public static Command pivotHigh(IntakePivot intakePivot) {
    return new IntakePivotHigh(intakePivot).withName("IntakePivotHigh");
  }

  public static Command rollerForward(IntakeRoller intakeRoller) {
    return Commands.runEnd(
        () -> intakeRoller.setState(IntakeRollerTarget.Forward),
        () -> intakeRoller.setState(IntakeRollerTarget.Off),
        intakeRoller);
  }

  public static Command rollerReverse(IntakeRoller intakeRoller) {
    return Commands.runEnd(
        () -> intakeRoller.setState(IntakeRollerTarget.Reverse),
        () -> intakeRoller.setState(IntakeRollerTarget.Off),
        intakeRoller);
  }
}
