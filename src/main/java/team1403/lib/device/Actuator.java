package team1403.lib.device;

/**
 * Marker interface denoting that a device is an actuator.
 *
 * <p>Actuators interact with the physical environment.
 * This has safety implications so actuators become disabled
 * when the robot is disabled.
 */
public interface Actuator extends Device {
}
