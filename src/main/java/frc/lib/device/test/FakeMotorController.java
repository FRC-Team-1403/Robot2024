package frc.lib.device.test;

import frc.lib.device.AdvancedMotorController;
import frc.lib.device.BaseDevice;
import frc.lib.device.CurrentSensor;
import frc.lib.device.Encoder;
import frc.lib.device.MotorController;

/**
 * A fake MotorController for testing.
 */
@SuppressWarnings("PMD.DataClass")
public class FakeMotorController extends BaseDevice implements AdvancedMotorController {
  /**
   * Constructor.
   *
   * @param name   The name for this instance.
   */
  public FakeMotorController(String name) {
    this(name, (Encoder) null, (CurrentSensor) null);
  }

  /**
   * Full Constructor.
   *
   * @param name    The name for this instance.
   * @param encoder The embedded encoder or null if none.
   * @param current The embedded current sensor or null if none.
   */
  public FakeMotorController(String name,
      Encoder encoder, CurrentSensor current) {
    super(name);
    m_encoder = encoder;
    m_currentSensor = current;
  }

  /**
   * Returns number of times {@link #stopMotor} was called.
   *
   * @return number of stopMotor() calls since constructed.
   */
  public final int countStopMotorCalls() {
    return m_stopCalls;
  }

  /**
   * Returns the last motor controller passed to {@link #follow}.
   *
   * @return null if follow() hasnt been called yet.
   */
  public MotorController getFollowing() {
    return m_follow;
  }

  /**
   * Returns last voltage to {@link #setVoltage}.
   *
   * @return NaN if setVoltage not yet called.
   */
  public double getVoltage() {
    return m_voltage;
  }

  /**
   * Returns last speed to {@link #setSpeed}.
   *
   * @return NaN if setSpeed not yet called.
   */
  public double getSpeed() {
    return m_speed;
  }

  /**
   * Returns the last mode to {@link #setMode}.
   *
   * @return null if setMode not yet called.
   */
  public CougarIdleMode getMode() {
    return m_mode;
  }

  /**
   * Returns the last ramp rate to {@link #setRampRate}.
   *
   * @return NaN if setRampRate not yet called.
   */
  public double getRampRate() {
    return m_rampRate;
  }

  /**
   * Returns the last p to {@link #setGains}.
   *
   * @return NaN if setGains not yet called.
   */
  public double getkP() {
    return m_kp;
  }

  /**
   * Returns the last i to {@link #setGains}.
   *
   * @return NaN if setGains not yet called.
   */
  public double getkI() {
    return m_ki;
  }

  /**
   * Returns the last D to {@link #setGains}.
   *
   * @return NaN if setGains not yet called.
   */
  public double getkD() {
    return m_kd;
  }

  /**
   * Returns the last position to {@link #setPosition}.
   *
   * @return NaN if setPosition not yet called.
   */
  public double getPosition() {
    return m_position;
  }

  /**
   * Returns the last current limit to {@link #setCurrentLimit}.
   *
   * @return NaN if setCurrentLimit not yet called.
   */
  public double getCurrentLimit() {
    return m_currentLimit;
  }

  @Override
  public void setVoltageCompensation(double voltage) {
    m_voltage = voltage;
  }

  @Override
  public void setSpeed(double speed) {
    m_speed = speed;
  }

  @Override
  public void stopMotor() {
    m_speed = 0;
    ++m_stopCalls;
  }

  @Override
  public void setInverted(boolean isInverted) {
    m_inverted = isInverted;
  }

  @Override
  public boolean getInverted() {
    return m_inverted;
  }

  @Override
  public void setRampRate(double rate) {
    this.m_rampRate = rate;
  }

  @Override
  public void setIdleMode(CougarIdleMode mode) {
    this.m_mode = mode;
  }

  @Override
  public void setPosition(double position) {
    this.m_position = position;
  }

  @Override
  public void setPidGains(double p, double i, double d) {
    this.m_kp = p;
    this.m_ki = i;
    this.m_kd = d;
  }

  @Override
  public void setAmpLimit(double amps) {
    this.m_currentLimit = amps;
  }

  @Override
  public Encoder getEmbeddedEncoder() {
    if (m_encoder != null) {
      return m_encoder;
    }
    return AdvancedMotorController.super.getEmbeddedEncoder();
  }

  @Override
  public CurrentSensor getEmbeddedCurrentSensor() {
    if (m_currentSensor != null) {
      return m_currentSensor;
    }
    return AdvancedMotorController.super.getEmbeddedCurrentSensor();
  }

  @Override
  public void follow(AdvancedMotorController source) {
    m_follow = source;
  }

  private double m_speed = Double.NaN;
  private double m_voltage = Double.NaN;
  private CougarIdleMode m_mode = CougarIdleMode.COAST;
  private double m_rampRate = Double.NaN;
  private double m_kp = Double.NaN;
  private double m_ki = Double.NaN;
  private double m_kd = Double.NaN;
  private double m_position = Double.NaN;
  private double m_currentLimit = Double.NaN;
  private int m_stopCalls;
  private AdvancedMotorController m_follow;
  private boolean m_inverted;
  private final Encoder m_encoder;
  private final CurrentSensor m_currentSensor;
}
