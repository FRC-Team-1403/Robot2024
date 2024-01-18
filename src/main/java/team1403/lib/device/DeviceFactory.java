package team1403.lib.device;

import com.revrobotics.SparkRelativeEncoder;

/**
 * Creates devices.
 *
 * <p>Subsystems create their devices through this factory.
 */
public interface DeviceFactory {
  /**
   * Creates a PowerDistributor for monitoring the power panel.
   *
   * <p>This is implicitly the default distributor. In practice we
   * only have one so this is assumed to be the standard CAN bus wiring.
   *
   * @param name   The name to give the distributor.
 
   * @return new PowerDistributor.
   */
  public PowerDistributor makePowerDistributor(String name);

  /**
   * Creates a Brushed CANSparkMax MotorController.
   *
   * @param name        The name of the new device instance.
   * @param channel     The CAN bus channel the motor controller is on.
   * @param encoderType The type of encoder used with the motor controller
   *
   * @return a new MotorController for a CANSparkMax.
   */
  public AdvancedMotorController makeBrushedCanSparkMax(String name, int channel, 
          SparkRelativeEncoder.Type encoderType);

  /**
   * Creates a Brusheless CANSparkMax MotorController.
   *
   * @param name        The name of the new device instance.
   * @param channel     The CAN bus channel the motor controller is on.
   * @param encoderType The type of encoder used with the motor controller
   *
   * @return a new MotorController for a CANSparkMax.
   */
  public AdvancedMotorController makeBrushlessCanSparkMax(String name, int channel,
          SparkRelativeEncoder.Type encoderType);

  /**
   * Creates a TalonSrx MotorController.
   *
   * @param name    The name of the n w device instance.
   * @param channel The CAN bus channel the motor controller is on.
   *
   * @return a new MotorController for a TalonSRX.
   */
  public AdvancedMotorController makeTalonSrx(String name, int channel);

  /**
   * Creates a TalonFX MotorController.
   *
   * @param name    The name of the new device instance.
   * @param deviceNumber  The CAN bus channel the motor controller is on.
   * @param mode  The control mode for the TalonFX
   *
   * @return a new MotorController for a TalonFX.
   */
  public AdvancedMotorController makeCougarTalonFx(String name, int deviceNumber);

  /**
   * Creates a VictorSpPwm MotorController.
   *
   * @param name    The name of the new device instance.
   * @param channel The CAN bus channel the motor controller is on.
   *
   * @return a new MotorController for a VictorSp.
   */
  public MotorController makeVictorSpPwm(String name, int channel);

  /**
   * Creates a VictorSpx MotorController.
   *
   * @param name    The name of the new device instance.
   * @param channel The CAN bus channel the motor controller is on.
   *
   * @return a new MotorController for a VictorSpx.
   */
  public AdvancedMotorController makeVictorSpx(String name, int channel);

  /**
   * Creates a limit switch.
   *
   * @param name    The name of the new device instance.
   * @param channel The robotRIO port the limit switch is on.
   *
   * @return a new LimitSwitch for a WPILib LimitSwitch.
   */
  public LimitSwitch makeLimitSwitch(String name, int channel);

  /**
  * Creates a double solenoid.
  *
  * @param name           The name of the new device instance.
  * @param forwardChannel The port in which the forwardChannel is being
  *                       controlled.
  * @param reverseChannel The port in which the reverseChannel is being
  *                       controlled.
  * 
  * @return a new DoubleSolenoid for a WPILib DoubleSolenoid
  */
  public CougarDoubleSolenoid makeDoubleSolenoid(String name,
            int forwardChannel, int reverseChannel);

  /**
   * Creates an analog device.
   *
   * @param name    The name of the new device instance.
   * @param channel The robotRIO port the analog device is on.
   *
   * @return a new AnalogDevice for a WPILib AnalogDevice.
   */
  public AnalogDevice makeAnalogDevice(String name, int channel);
}
