package team1403.lib.device;

/**
 * A device adapts some hardware component for software control.
 *
 * <p>The useful interface for a device depends on the particular type of
 * device so are specialized. This base interface is more a marker to claim
 * a class is a Device.
 * 
 * <p>Devices have names for traceability.
 */
public interface Device {
  /**
   * All CougarRobot devices have names.
   *
   * @return The name given to this device instance.
   */
  public String getName();
}
