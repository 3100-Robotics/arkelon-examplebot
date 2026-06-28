package frc.robot.logging.configuration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Configurator {
  // TODO whats this library called
  private Runnable preConfig =
      () ->
          DriverStation.reportError(
              "preConfig has not been set. This will probably leak memory over time in your robot program. This is likely not a problem with your code, but is likely a problem with the library",
              false);
  private Runnable postConfig = () -> {};
  private Runnable configureCallback =
      () ->
          DriverStation.reportWarning(
              "Unset logging configurator. Please call call setConfigureCallback()", false);
  private final EventLoop eventLoop = new EventLoop();
  private final List<Hook> hooks = new ArrayList<>();

  public Configurator(Runnable preConfig, Runnable postConfig) {
    this.preConfig = preConfig;
    this.postConfig = postConfig;
  }

  /**
   * Adds a hook which when the value changes will trigger reconfiguration of logging
   *
   * @see {@link Configurator#setConfigureCallback}
   * @param hookValueGetter supplier of a value
   * @return RootLogging instance for chaining
   */
  public Configurator addHook(Supplier<?> hookValueGetter) {
    Hook newHook = new Hook(hookValueGetter);
    eventLoop.bind(newHook::check);
    hooks.add(newHook);
    return this;
  }

  /**
   * Set the user configuration function
   *
   * @param configureCallback
   * @return
   */
  public Configurator setConfigureCallback(Runnable configureCallback) {
    this.configureCallback =
        () -> {
          System.out.println("Configuring RootLogger");
          preConfig.run();
          configureCallback.run();
          postConfig.run();
        };
    return this;
  }

  public Runnable getConfiguratorCallback() {
    return configureCallback;
  }

  public void checkAllHooks() {
    eventLoop.poll();
    hooks.forEach(
        hook -> {
          if (hook.fired) {
            configureCallback.run();
            hook.reset();
          }
        });
  }
}
