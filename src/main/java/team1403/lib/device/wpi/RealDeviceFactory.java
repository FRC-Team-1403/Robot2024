package team1403.lib.device.wpi;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import team1403.lib.device.AdvancedMotorController;
import team1403.lib.device.AnalogDevice;
import team1403.lib.device.CougarAccelerometer;
import team1403.lib.device.CougarDoubleSolenoid;
import team1403.lib.device.DeviceFactory;
import team1403.lib.device.LimitSwitch;
import team1403.lib.device.MotorController;
import team1403.lib.device.PowerDistributor;


/**
 * A DeviceFactory that returns devices implemented using WPILibrary components.
 *
 * <p>These are real devices.
 */
@SuppressWarnings({"PMD.TooManyMethods"})
public class RealDeviceFactory implements DeviceFactory {
  /**
   * Returns a WpiBuiltinAccelerometer.
   */
  @Override
  public CougarAccelerometer makeBuiltinAccelerometer(
      String name, CougarAccelerometer.Range range) {
    return new WpiBuiltinAccelerometer(name, range);
  }

  /**
   * Returns a WpiPowerDistribution instance.
   */
  @Override
  public PowerDistributor makePowerDistributor(String name) {
    return new WpiPowerDistribution(name);
  }

  /**
   * Returns a Brushless CanSparkMax instance.
   */
  @Override
  public AdvancedMotorController makeBrushlessCanSparkMax(
      String name, int channel, SparkRelativeEncoder.Type encoderType) {
    return CougarSparkMax.makeBrushless(name, channel, encoderType);
  }

  /**
   * Returns a Brushed CanSparkMax instance.
   */
  @Override
  public AdvancedMotorController makeBrushedCanSparkMax(
      String name, int channel, SparkRelativeEncoder.Type encoderType) {
    return CougarSparkMax.makeBrushed(name, channel, encoderType);
  }

  /**
   * Returns a TalonSrx instance.
   */
  @Override
  public AdvancedMotorController makeTalonSrx(String name, int channel) {
    return new TalonSrx(name, channel);
  }

  /**
   * Returns a VictorSP instance.
   */
  @Override
  public MotorController makeVictorSpPwm(
      String name, int channel) {
    return new VictorSp(name, channel);
  }

  /**
   * Returns a VictorSpx instance.
   */
  @Override
  public AdvancedMotorController makeVictorSpx(String name, int channel) {
    return new VictorSpx(name, channel);
  }

  @Override

  public AdvancedMotorController makeCougarTalonFx(String name, int deviceNumber) {

    return new CougarTalonFx(name, deviceNumber);
  }

  /**
   * Returns a WpiLimitSwitch instance.
   */
  @Override
  public LimitSwitch makeLimitSwitch(String name, int channel) {
    return new WpiLimitSwitch(name, channel);
  }

  /**
   * Returns a WpiDoubleSolenoid instance.
   */
  @Override
  public CougarDoubleSolenoid makeDoubleSolenoid(
      String name, int forwardChannel, int reverseChannel) {
    return new WpiDoubleSolenoid(name, forwardChannel, reverseChannel);
  }

  /**
   * Returns a WpiAnalogDevice instance.
   */
  @Override
  public AnalogDevice makeAnalogDevice(String name, int channel) {
    return new WpiAnalogDevice(name, channel);
  }
}
