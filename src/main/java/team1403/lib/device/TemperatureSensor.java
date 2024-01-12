package team1403.lib.device;

/**
 * TemperatureSensor for detecting temperature.
 */
public interface TemperatureSensor extends Sensor {
  /**
   * Returns the current temperature in degrees C.
   *
   * @return degrees C
   */
  double getCelsius();

  /**
   * Returns the current temperature in degress F.
   *
   * @return degrees F
   */
  default double getFarenheit() {
    return toFarenheit(getCelsius());
  }

  /**
   * Convert from celsius into farenheit.
   *
   * @param celsius desired temperature
   * @return degrees F
   */
  public static double toFarenheit(double celsius) {
    return 32.0 + celsius * 9.0 / 5.0;
  }
}
