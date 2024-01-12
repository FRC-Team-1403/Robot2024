package team1403.lib.device;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * A DoubleSolenoid is used to control valves in a pneumatics system.
 */
public interface CougarDoubleSolenoid extends Actuator {
  /**
   * Set the state of the DoubleSolenoid.
   */
  public void setState(Value value);

  /**
   * Toggle the state of the DoubleSolenoid.
   *
   * <p>If the DoubleSolenoid is in a forward state, 
   * then it will switch to a reverse state.
   *
   * <p>If the DoubleSolenoid is in a reverse state, 
   * then it will switch to a forward state.
   *
   * <p>If the state of the DoubleSolenoid is off, then nothing will happen.
   */
  public void toggle();

  /**
   * Returns the state of the DoubleSolenoid.
   *
   * @return The state the DoubleSolenoid is currently in.
   */
  public Value getState();
}
