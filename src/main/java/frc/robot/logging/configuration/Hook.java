package frc.robot.logging.configuration;

import java.util.function.Supplier;

public class Hook {
  public final Supplier<?> getHookValue;
  public boolean fired = false;
  private Object last;

  public Hook(Supplier<?> getHookValue) {
    this.getHookValue = getHookValue;
    last = getHookValue.get();
  }

  public void check() {
    if (getHookValue.get() != last) {
      fired = true;
    }
    last = getHookValue.get();
  }

  public void reset() {
    fired = false;
  }
}
