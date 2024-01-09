package team1403.lib.device;

/**
 * LimitSwtch
 *
 * <p>Denotes a limit switch.
 */
public interface LimitSwitch extends Sensor {
  /**
   * Determine whether the switch is currently triggered or not.
   *
   * @return true if the limit switch is triggered, false if not.
   */
  public boolean isTriggered();
}
