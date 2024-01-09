package team1403.lib.device;

/**
 * Basic Motor Controller class that only implements basic motor control methods. 
 */
public interface MotorController extends Actuator {

  /**
   * Set the relative speed.
   *
   * @param speed The ratio of speed compared to full.
   *              This is a range -1.0 .. 1.0. The actual
   *              speed depends on the motor and battery level.
   */
  void setSpeed(double speed);

  /**
   * Set whether the motor is inverted or not. Inverted motors run backward.
   *
   * @param isInverted true if the motor is inverted.
   */
  void setInverted(boolean isInverted);

  /**
   * Returns whether motor is inverted or not.
   *
   * @return true if the motor was setInverted. False by default.
   */
  boolean getInverted();

  /**
   * Force the motor to stop.
   *
   * <p>This is potentially different from setting the speed to 0 as an
   * intentional stop may be treated differently within the motor.
   */
  void stopMotor();
  
}
