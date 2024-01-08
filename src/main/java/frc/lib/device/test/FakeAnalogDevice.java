package frc.lib.device.test;

import frc.lib.device.AnalogDevice;
import frc.lib.device.BaseDevice;

/**
 * Implements a fake analog device.
 */

public class FakeAnalogDevice extends BaseDevice  implements AnalogDevice  {

  /**
   * Constructor.
   *
   * @param name The name of the instance.
   */

  public FakeAnalogDevice(String name) {
    super(name);
    m_name = name;
    
  }

  @Override
  public String getName() {
    return m_name;
  }
  
  /**
   * Set the analog value to be returned.
   *
   * @param value of the device to be returned
   */

  public void setValue(double value) {
    m_value = value;
  }

  @Override
  public double getAnalogValue() {
    return 0;
  }
 
  double m_value = 0.0;
  private final String m_name;
}
  


