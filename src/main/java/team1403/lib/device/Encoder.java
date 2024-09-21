package team1403.lib.device;

/**
 * Represents an encoder.
 */
public interface Encoder extends Sensor {
  /**
   * Returns number of ticks per revolution within the encoder.
   *
   * @return value determined by underlying encoder.
   */
  double ticksPerRevolution();

  /**
   * Get the position of the encoder.
   *
   * @return The angle of a tick is determined by the encoder.
   *
   * @see ticksPerRevolution
   */
  double getPositionValue();

  /**
   * Set conversion factor for position ticks.
   */
  void setPositionConversionFactor(double conversionFactor);

  /**
   * Set conversion factor for velocity.
   *
   * @param conversionFactor The conversion factor to be used with velocity.
   */
  void setVelocityConversionFactor(double conversionFactor);

  /**
   * Get the rate of rotation.
   *
   * @return rate of encoder rotation
   */
  double getVelocityValue();

  /**
   * Resets the position of the encoder to the given value.
   */
  void setPosition(double position);
}
