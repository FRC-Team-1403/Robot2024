package team1403.lib.device;

/**
 * CurrentSensor for detecting current within a device.
 */
public interface CurrentSensor extends Sensor {
  /**
   * Gets the current.
   *
   * @return amps.
   */
  double getAmps();
}
