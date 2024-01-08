package frc.lib.device.wpi;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.lib.device.AdvancedMotorController;

/**
 * Device implementation for a WPI_VictorSPX motor controller.
 */
public class VictorSpx extends WPI_VictorSPX implements AdvancedMotorController {  
  /**
   * Constructor.
   *
   * @param name The name name for the device.
   * @param channel The CAN channel the motor is on.
   */
  public VictorSpx(String name, int channel) {
    super(channel);
    m_name = name;
  }

  /**
   * Return the WPI_VictorSPX API so we can do something specific.
   *
   * @return The underlying {@code WPI_VictorSPX} instance.
   */
  public final WPI_VictorSPX getVictorSpxApi() {
    return this;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  /**
   * Follow another VictorSpx motor.
   *
   * @param source Must be a com.ctre.phoenix.motorcontrol.IMotorController
   *               (e.g. another VictorSpx or TalonSpx)
   *
   * @throws ClassCastException if motor is not compatible.
   */
  @Override
  public void follow(AdvancedMotorController source) {
    super.follow((IMotorController)source);  // Will throw an exception if source is not compatible.
  }

  @Override
  public void setVoltageCompensation(double voltage) {
    super.configVoltageCompSaturation(voltage);
  }

  @Override
  public final void setSpeed(double speed) {
    super.set(speed);
  }

  @Override
  public void setPosition(double position) {
  }

  @Override 
  public void setPidGains(double p, double i, double d) {
  }

  @Override
  public void setIdleMode(CougarIdleMode mode) {
  }

  @Override
  public void setRampRate(double rate) {
    configClosedloopRamp(rate);
  }
  
  @Override
  public void setAmpLimit(double amps) {
  }
  
  private final String m_name;
}
