package team1403.lib.device.wpi;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;

// Note this is different than wpilibj.interfaces.Accelerometer
import team1403.lib.device.CougarAccelerometer;

/**
 * Provides access to the RoboRIO Builtin Accelerometer.
 */
public class WpiBuiltinAccelerometer extends BuiltInAccelerometer
                                     implements CougarAccelerometer {
  /**
   * Constructor.
   *
   * @param name The device name.
   * @param range The sensitivity range.
   */
  public WpiBuiltinAccelerometer(String name, Range range) {
    super(range);
    m_name = name;
    m_range = range;
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public void setRange(Range range) {
    super.setRange(range);
    m_range = range;
  }

  @Override
  public Range getRange() {
    return m_range;
  }

  private final String m_name;
  private Range m_range;
}
