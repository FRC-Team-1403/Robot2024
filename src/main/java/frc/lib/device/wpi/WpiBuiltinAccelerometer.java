package frc.lib.device.wpi;


import edu.wpi.first.wpilibj.BuiltInAccelerometer;

// Note this is different than wpilibj.interfaces.Accelerometer
import frc.lib.device.CougarAccelerometer;

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
  public WpiBuiltinAccelerometer(String name,Range range) {
    super();
    m_name = name;
    m_range = range;
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public void setRange(edu.wpi.first.wpilibj.interfaces.Accelerometer.Range range) {
    super.setRange(range);
    m_range = range;
  }

  @Override
  public edu.wpi.first.wpilibj.interfaces.Accelerometer.Range getRange() {
    return m_range;
  }

  private final String m_name;
  private edu.wpi.first.wpilibj.interfaces.Accelerometer.Range m_range;
}
