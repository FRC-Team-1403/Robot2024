package team1403.lib.device;

/**
 * Marker interface denoting that a device is a sensor.
 *
 * <p>Sensors read values from the environment but do not tend to interact
 * with the environment. Given the passive nature of sensors, they may
 * still be active when the robot is disabled allowing the robot to continue
 * sensing its environment even though it cannot move or respond to what it
 * observes.
 */
public interface Sensor extends Device {
}
