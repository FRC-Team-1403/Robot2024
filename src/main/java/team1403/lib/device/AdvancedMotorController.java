package team1403.lib.device;

/**
 * A MotorController controls a robot motor.
 *
 * <p>Motors often have far more intricate interfaces, particularly
 * for configuring them. This interface is only intended to say general
 * things, particularly to control the motor. You may need to access
 * the concrete class specialized for a particular motor controller to
 * get a suitable interface to configure it.
 *
 * <p>MotorControllers often have additional embedded sensors.
 * These can be obtained through the interface where available.
 * If the controller does not support the embedded sensor then attempting
 * to access them will throw a {@link NoSuchDeviceError}. These are considered
 * programming errors assuming that you require those embedded sensors.
 * If the sensors are not required, then check their availability with the
 * {@code has} methods before requesting it to avoid the exception.
 */
public interface AdvancedMotorController extends MotorController {

  /**
   * Represents the neutral mode of the motor.
   */
  public enum CougarIdleMode {
    /**
     * Represents idle mode in brake mode.
     */
    BRAKE,
    /**
     * Represents idle mode in coast mode.
     */
    COAST
  }

  /**
   * Configure this motor to follow another.
   *
   * @param source The source motor controller that this should follow.
   */
  void follow(AdvancedMotorController source);

  /**
   * Sets the voltage for the motor independent of battery level.
   *
   * @param voltage The desired voltage.
   */
  void setVoltageCompensation(double voltage);

  /**
   * Set the output value of the motor in encoder ticks or an analog value.
   *
   * @param position The setpoint value in encoder ticks/analog value.
   */
  void setPosition(double position);

  /**
   * Set the gains for the motor.
   */
  void setPidGains(double p, double i, double d);

  /**
   * Set the idle mode for the motor.
   *
   * @param mode the idle mode of the motor
   */
  void setIdleMode(CougarIdleMode mode);

  /**
   * Sets how quickly the motor will increase/decrease speed. 
   *
   * @param rate Defines the seconds needed to reach a value.
   */
  void setRampRate(double rate);

  /**
   * Sets the current limit.
   *
   * @param amps The max current
   */
  void setAmpLimit(double amps);

  /**
   * Checks if the MotorController has an embedded encoder.
   *
   * @return true if it does, false if not.
   */
  default boolean hasEmbeddedEncoder() {
    try {
      getEmbeddedEncoder();
      return true;
    } catch (NoSuchDeviceError err) {
      return false;
    }
  }

  /**
   * Returns the embeded controller.
   *
   * <p>If you do not require an encoder, then call {@link hasEmbeddedEncoder}
   * to see if there is an encoder available. Otherwise, if you do require
   * an encoder, then this check is not necessary because the check here
   * will reinforce your requirement.
   *
   * @return encoder embedded within the motor controller.
   *
   * @throws NoSuchDeviceError if there is no encoder.
   * @see hasEmbeddedEncoder
   */
  default Encoder getEmbeddedEncoder() {
    throw new NoSuchDeviceError(
        String.format("Device '%s' does not have an embedded encoder.",
            getName()));
  }

  /**
   * Checks if the MotorController has an embedded current sensor.
   *
   * @return true if it does, false if not.
   */
  default boolean hasEmbeddedCurrentSensor() {
    try {
      getEmbeddedCurrentSensor();
      return true;
    } catch (NoSuchDeviceError err) {
      return false;
    }
  }

  /**
   * Returns the embeded current sensor.
   *
   * <p>If you do not require this sensor, then call {@link hasEmbeddedEncoder}
   * to see if there is one available. Otherwise, if you do require
   * the sensor, then this check is not necessary because the check here
   * will reinforce your requirement.
   *
   * @return CurrentSensor embedded within MotorController.
   *
   * @throws NoSuchDeviceError if there is no embedded current sensor.
   * @see hasEmbeddedEncoder
   */
  default CurrentSensor getEmbeddedCurrentSensor() {
    throw new NoSuchDeviceError(
        String.format("Device '%s' does not have an embeded current sensor.",
            getName()));
  }
}
