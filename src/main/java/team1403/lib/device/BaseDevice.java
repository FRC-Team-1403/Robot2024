package team1403.lib.device;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A base class for a device.
 *
 * <p>Note that many devices may not be derived from this
 * class. Devices only need to implement the Device interface.
 */
public class BaseDevice implements Device, Sendable {
  /**
   * Constructor.
   *
   * @param name The name of the device instance
   *             should be unique among all instances.
   */
  public BaseDevice(String name) {
    m_name = name;
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Unknown Device");
    builder.addStringProperty("Name", this::getName, null);
  }

  private final String m_name;
}
