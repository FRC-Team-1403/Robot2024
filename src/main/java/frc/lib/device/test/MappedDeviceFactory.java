package frc.lib.device.test;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import frc.lib.device.AdvancedMotorController;
import frc.lib.device.AnalogDevice;
import frc.lib.device.CougarDoubleSolenoid;
import frc.lib.device.Device;
import frc.lib.device.DeviceFactory;
import frc.lib.device.LimitSwitch;
import frc.lib.device.MotorController;
import frc.lib.device.NoSuchDeviceError;
import frc.lib.device.PowerDistributor;

/**
 * A device factory for testing.
 *
 * <p>A MappedDeviceFactory is pre-populated with
 * the devices to return when asked for by name. It
 * is meant for testing. Devices are removed from the
 * map as they are returned since names are supposed
 * to be unique.
 */
@SuppressWarnings({"PMD.UseConcurrentHashMap", "PMD.TooManyMethods"})
public class MappedDeviceFactory implements DeviceFactory {
  /**
   * Update the factory with all the components of the other factory.
   *
   * @param other The other factory supplying additional components
   *              must have disjoint components in it.
   *
   * @throws IllegalArgumentException if other factory has keys in use.
   */
  public void update(MappedDeviceFactory other) {
    for (Device device : other.m_deviceMap.values()) {
      putDevice(device);
    }
  }

  /**
   * Adds a MotorController.
   *
   * @param controller The MotorController to return when asked for.
   */
  public void putMotorController(MotorController controller) {
    putDevice(controller);
  }

  /**
   * Adds a limit switch.
   *
   * @param limitSwitch The limit switch to return when asked for.
   */
  public void putLimitSwitch(LimitSwitch limitSwitch) {
    putDevice(limitSwitch);
  }

  /**
   * Populates the factory with a device.
   *
   * @param device The device must have a unique name.
   */
  public void putDevice(Device device) {
    final String name = device.getName();
    if (m_deviceMap.containsKey(name)) {
      throw new IllegalArgumentException(
          String.format("Device '%'s already exists", name));
    }
    m_deviceMap.put(name, device);
  }

  /**
   * Lookup and return a device.
   *
   * <p>The device will removed from the factory and return
   * {@code null} next time.
   *
   * @param name The desired device name.
   * @return The device previously added with {@link #putDevice} or variant.
   *
   * @throws NoSuchDeviceError if the name is not (or no longer)
   *                           known to the factory.
   */
  public Device takeDevice(String name) {
    if (!m_deviceMap.containsKey(name)) {
      throw new NoSuchDeviceError(
          String.format("Unexpected device '%s'", name));
    }
    return m_deviceMap.remove(name);
  }

  /**
   * Returns the remaining devices as a human-readable string.
   *
   * <p>This is to support writing tests that fail if there are devices left
   * over. The test can match this string against "" and show what was left
   * over as it fails.
   *
   * @return String form of enumerated devices.
   */
  public String remainingDeviceNamesToString() {
    if (m_deviceMap.isEmpty()) {
      return "";
    }
    return m_deviceMap.keySet().toString();
  }

  /**
   * Return the populated devices.
   *
   * <p>This is intended to allow tests to discover what has not been asked for.
   *
   * @return Map of registered device names to their instances.
   */
  public Map<String, Device> getRemainingDevices() {
    return m_deviceMap;
  }

  /**
   * Provides the list of parameters passed when asking for devices.
   *
   * <p>This is intended to allow tests to verify the parameters given to
   * the factory when creating devices. For example the channel that the
   * device is wired to (if applicable).
   *
   * @return Map of registered device names to the parameter list of the factory
   *         method used to create the device.
   */
  public Map<String, List<Object>> getCalls() {
    return m_calls;
  }

  @Override
  public CougarAccelerometer makeBuiltinAccelerometer(
      String name, CougarAccelerometer.Range range) {
    m_calls.put(name, Arrays.asList(name, range));
    return (CougarAccelerometer)takeDevice(name);
  }

  @Override
  public PowerDistributor makePowerDistributor(String name) {
    m_calls.put(name, Arrays.asList(name));
    return (PowerDistributor)takeDevice(name);
  }

  @Override
  public AdvancedMotorController makeBrushlessCanSparkMax(
      String name, int channel, SparkMaxRelativeEncoder.Type encoderType) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel), encoderType));
    return (AdvancedMotorController)takeDevice(name);
  }

  @Override
  public AdvancedMotorController makeBrushedCanSparkMax(
      String name, int channel, SparkMaxRelativeEncoder.Type encoderType) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel), encoderType));
    return (AdvancedMotorController)takeDevice(name);
  }

  @Override
  public MotorController makeVictorSpPwm(
      String name, int channel) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel)));
    return (MotorController)takeDevice(name);    
  }

  @Override
  public AdvancedMotorController makeVictorSpx(
      String name, int channel) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel)));
    return (AdvancedMotorController)takeDevice(name);
  }

  @Override

  public AdvancedMotorController makeCougarTalonFx(String name,
      int deviceNumber) {


    m_calls.put(name, Arrays.asList(name, deviceNumber));
    return (AdvancedMotorController)takeDevice(name);
  }

  @Override
  public AdvancedMotorController makeTalonSrx(
      String name, int channel) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel)));
    return (AdvancedMotorController)takeDevice(name);
  }

  @Override
  public LimitSwitch makeLimitSwitch(String name, int channel) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel)));
    return (LimitSwitch)takeDevice(name);
  }

  @Override
  public CougarDoubleSolenoid makeDoubleSolenoid(
      String name, int forwardChannel, 
      int reverseChannel) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(forwardChannel), 
                Integer.valueOf(reverseChannel)));
    return (CougarDoubleSolenoid)takeDevice(name);
  }

  @Override
  public AnalogDevice makeAnalogDevice(String name, int channel) {
    m_calls.put(name, Arrays.asList(name, Integer.valueOf(channel)));
    return (AnalogDevice)takeDevice(name);
  }
  
  private final Map<String, Device> m_deviceMap = new HashMap<>();
  private final Map<String, List<Object>> m_calls = new HashMap<>();
}
