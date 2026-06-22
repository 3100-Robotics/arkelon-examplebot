package frc.robot.logging;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LogXboxController implements RunsPeriodic {
  CommandXboxController controller;
  XboxController hid;
  final String name;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable dsRoot = inst.getTable("Controllers");
  NetworkTable controllerRoot;

  NetworkTable axes;
  NetworkTable buttons;
  NetworkTable povs;

  BooleanPublisher a;
  BooleanPublisher b;
  BooleanPublisher x;
  BooleanPublisher y;

  BooleanPublisher lb;
  BooleanPublisher rb;

  BooleanPublisher ls;
  BooleanPublisher rs;

  BooleanPublisher back;
  BooleanPublisher start;

  IntegerPublisher pov;

  DoublePublisher lx;
  DoublePublisher rx;
  DoublePublisher ly;
  DoublePublisher ry;

  DoublePublisher lt;
  DoublePublisher rt;

  public LogXboxController(String name, Integer slot, CommandXboxController controller) {
    this.name = "Controller " + name + ", slot:" + slot.toString();
    this.controller = controller;
    this.hid = controller.getHID();

    controllerRoot = dsRoot.getSubTable(this.name);

    axes = controllerRoot.getSubTable("Axes");
    buttons = controllerRoot.getSubTable("Buttons");
    povs = controllerRoot.getSubTable("POVs");

    a = buttons.getBooleanTopic("A").publish();
    b = buttons.getBooleanTopic("B").publish();
    x = buttons.getBooleanTopic("X").publish();
    y = buttons.getBooleanTopic("Y").publish();

    lb = buttons.getBooleanTopic("LeftBumper").publish();
    rb = buttons.getBooleanTopic("RightBumper").publish();

    back = buttons.getBooleanTopic("Back").publish();
    start = buttons.getBooleanTopic("Start").publish();

    ls = buttons.getBooleanTopic("LeftStick").publish();
    rs = buttons.getBooleanTopic("RightStick").publish();

    pov = povs.getIntegerTopic("DPad").publish();

    lx = axes.getDoubleTopic("LeftX").publish();
    rx = axes.getDoubleTopic("RightX").publish();
    ly = axes.getDoubleTopic("LeftY").publish();
    ry = axes.getDoubleTopic("RightY").publish();

    lt = axes.getDoubleTopic("LeftTrigger").publish();
    rt = axes.getDoubleTopic("RightTrigger").publish();
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void periodic() {
    a.set(hid.getAButton());
    b.set(hid.getBButton());
    x.set(hid.getXButton());
    y.set(hid.getYButton());
    lb.set(hid.getLeftBumperButton());
    rb.set(hid.getRightBumperButton());
    back.set(hid.getBackButton());
    start.set(hid.getStartButton());
    ls.set(hid.getLeftStickButton());
    rs.set(hid.getRightStickButton());

    pov.set(hid.getPOV());

    lx.set(hid.getLeftX());
    rx.set(hid.getRightX());
    ly.set(hid.getLeftY());
    ry.set(hid.getRightY());

    lt.set(hid.getLeftTriggerAxis());
    rt.set(hid.getRightTriggerAxis());
  }
}
