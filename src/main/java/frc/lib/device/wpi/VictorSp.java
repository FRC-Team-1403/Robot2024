package frc.lib.device.wpi;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import frc.lib.device.MotorController;

/**
 * Device implementation for a VictorSp motor controller.
 */
public class VictorSp extends VictorSP
                       implements MotorController {
  /**
   * Constructor.
   *
   * @param name The name for the device.
   * @param channel The CAN channel the motor is on.
   */
  public VictorSp(String name, int channel) {
    super(channel);
    m_name = name;
  }

  /**
   * Return the WPI_VictorSP API so we can do something specific.
   *
   * @return The underlying {@code WPI_VictorSPX} instance.
   */
  public final VictorSP getVictorSpxApi() {
    return this;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  @Override
  public final void setSpeed(double speed) {
    super.set(speed);
  }

  private final String m_name;
}
