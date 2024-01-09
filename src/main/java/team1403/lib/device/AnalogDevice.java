package team1403.lib.device;

/**
 * AnalogDevice
 *
 * <p>Denotes an analog device.
 */

public interface AnalogDevice extends Sensor {

  /**
   * Gets the value of the analog device.
   *
   * @return double value, which is the sensor value.
   */
  
  double getAnalogValue();

}
