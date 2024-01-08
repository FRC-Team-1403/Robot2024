package frc.lib.device.test;

import frc.lib.device.BaseDevice;
import frc.lib.device.CougarAccelerometer;

/**
 * Implements a fake accelerometer.
 */
@SuppressWarnings({"PMD.DataClass", "MemberName"})
public class FakeCougarAccelerometer extends BaseDevice
                                     implements CougarAccelerometer {
  /**
   * Constructor.
   *
   * @param name The device name.
   * @param range The max sensitivity.
   */
  public FakeCougarAccelerometer(String name, Range range) {
    super(name);
    m_range = range;
  }

  /**
   * Set accelerations.
   *
   * @param x X axis
   * @param y Y axis
   * @param z Z axis
   */
  public void setXyz(double x, double y, double z) {
    m_x = x;
    m_y = y;
    m_z = z;
  }

  @Override
  public double getX() {
    return m_x;
  }

  @Override
  public double getY() {
    return m_y;
  }

  @Override
  public double getZ() {
    return m_z;
  }

  @Override
  public void setRange(Range range) {
    m_range = range;
  }

  @Override
  public Range getRange() {
    return m_range;
  }

  private double m_x = Double.NaN;  // NOPMD
  private double m_y = Double.NaN;  // NOPMD
  private double m_z = Double.NaN;  // NOPMD
  private Range m_range;
}
