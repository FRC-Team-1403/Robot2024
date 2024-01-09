package team1403.lib.device.wpi;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import team1403.lib.device.CougarDoubleSolenoid;

/**
 * Device implementation for a WpiDoubleSolenoid.
 */
public class WpiDoubleSolenoid extends DoubleSolenoid
                               implements CougarDoubleSolenoid {

  /**
   * Constructor.
   *
   * @param name The name for the device.
   * @param forwardChannel The port number for the forward channel.
   * @param reverseChannel The port number for the reverse channel.
   */
  public WpiDoubleSolenoid(String name, int forwardChannel, 
                           int reverseChannel) {
    super(PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
    m_name = name;
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public void setState(Value value) {
    super.set(value);
  }

  @Override
  public void toggle() {
    super.toggle();
  }

  @Override
  public Value getState() {
    return super.get();
  }

  private final String m_name;
}
