package team1403.lib.device.virtual;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;

import team1403.lib.device.BaseDevice;
import team1403.lib.device.LimitSwitch;


/**
 * A LimitSwitch provided by an injected function.
 *
 * <p>Wraps a LimitSwitch interface around a boolean function so it
 * can be used to act as a "virtual" LimitSwitch.
 */
public class LimitSwitchImpl extends BaseDevice implements LimitSwitch {
  /**
   * Constructor.
   *
   * @param name The device name for this instance.
   * @param isTriggeredFunc Returns true if the limitSwitch is triggered,
   *                        false otherwise.
   */
  public LimitSwitchImpl(String name, BooleanSupplier isTriggeredFunc) {
    super(name);
    m_func = isTriggeredFunc;
  }

  @Override
  public final boolean isTriggered() {
    return m_func.getAsBoolean();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("LimitSwitch");
    builder.addBooleanProperty("Triggered", m_func, null);
  }

  private final BooleanSupplier m_func;
}
