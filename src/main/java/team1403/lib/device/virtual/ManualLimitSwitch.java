package team1403.lib.device.virtual;

import edu.wpi.first.util.sendable.SendableBuilder;

import team1403.lib.device.BaseDevice;
import team1403.lib.device.LimitSwitch;


/**
 * A limit switch that is explicitly triggered through its API.
 */
public class ManualLimitSwitch
       extends BaseDevice
       implements LimitSwitch {
  /**
   * Constructor.
   *
   * @param name The device name.
   */
  public ManualLimitSwitch(String name) {
    super(name);
  }

  /**
   * Set the device as being actively triggered (or not).
   *
   * @param triggered Whether device state should be triggered or not.
   */
  public void setTriggered(boolean triggered) {
    m_triggered = triggered;
  }

  @Override
  public boolean isTriggered() {
    return m_triggered;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("LimitSwitch");
    builder.addBooleanProperty("Triggered", this::isTriggered, this::setTriggered);
  }

  private boolean m_triggered;
}
