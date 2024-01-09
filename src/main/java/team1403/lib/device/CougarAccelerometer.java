package team1403.lib.device;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;

/**
 * Accelerometer
 *
 * <p>Denotes an accelerometer. This will return acceleration in the
 * device's orientation.
 */
public interface CougarAccelerometer extends Accelerometer, Sensor {
  /**
   * Returns the maximum gforces that get(X|Y|Z) will return.
   *
   * @return The configured range limit.
   */
  Accelerometer.Range getRange();
}
