package team1403.lib.device.wpi;

import edu.wpi.first.wpilibj.AnalogInput;

import team1403.lib.device.AnalogDevice;

/**
 * Implements an Analog Device using WPI AnalogInput.
 */

public class WpiAnalogDevice extends AnalogInput implements AnalogDevice {

  /**
   * Constructor.
   *
   * @param name The name given to the analog device.
   * 
   * @param channel The RoboRIO channel the analog is connected to.
   */ 

  public WpiAnalogDevice(String name, int channel) {
    super(channel);
    m_name = name;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  @Override
  public final double getAnalogValue() {
    return getValue();
  }
 
  private final String m_name;
}

