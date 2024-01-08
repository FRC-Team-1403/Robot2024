package frc.lib.device.test;

import frc.lib.device.BaseDevice;
import frc.lib.device.Encoder;


/**
 * Implements a fake encoder.
 */
public class FakeEncoder extends BaseDevice implements Encoder {
  /**
   * Constructor.
   *
   * @param name The name of the instance.
   * @param ticksPerRevolution The fake's ticks per revolution is fixed.
   */
  public FakeEncoder(String name, double ticksPerRevolution) {
    super(name);
    m_ticksPerRevolution = ticksPerRevolution;
  }

  /**
   * Sets the current velocity to return.
   *
   * @param velocity The revolutions per minute.
   */
  public void setVelocity(double velocity) {
    m_velocity = velocity;
  }

  /**
   * Set the current positional ticks to return.
   *
   * @param ticks The number of ticks for the current position.
   */
  public void setPositionTicks(double ticks) {
    m_ticks = ticks;
  }

  /**
   * Sets the current positional ticks in terms of revolutions.
   *
   * @param revs The number of revolutions implies positional ticks.
   */
  public void setRevolutions(double revs) {
    m_ticks = revs * m_ticksPerRevolution;
  }

  @Override
  public double getPositionValue() {
    return m_ticks * m_positionConversionFactor;
  }

  @Override
  public double getVelocityValue() {
    return m_velocity;
  }

  @Override
  public double ticksPerRevolution() {
    return m_ticksPerRevolution;
  }

  @Override
  public void setPositionConversionFactor(double conversionFactor) {
    m_positionConversionFactor = conversionFactor;
  }

  @Override
  public void setVelocityConversionFactor(double conversionFactor) {
    m_velocityConversionFactor = conversionFactor;
  }

  @Override
  public void setPositionOffset(double position) {
    m_ticks = position;
  }

  private final double m_ticksPerRevolution;
  private double m_positionConversionFactor = 1.0;
  private double m_velocityConversionFactor = 1.0;
  private double m_ticks;
  private double m_velocity;
  
}
