package team1403.lib.device.wpi;

import edu.wpi.first.wpilibj.DigitalInput;

import team1403.lib.device.LimitSwitch;


/**
 * Implements a real limit switch.
 */
public class WpiLimitSwitch extends DigitalInput
                            implements LimitSwitch {
  /**
   * Constructor.
   *
   * @param name The name given to the switch device.
   * @param channel The RoboRIO channel the switch is connected to.
   */ 
  public WpiLimitSwitch(String name, int channel) {
    super(channel);
    m_name = name;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  @Override
  public final boolean isTriggered() {
    return get();
  }

  private final String m_name;
}
