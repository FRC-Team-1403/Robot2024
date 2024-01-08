package frc.lib.device.wpi;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.AnalogAccelerometer;
import frc.lib.device.AdvancedMotorController;
import frc.lib.device.AnalogDevice;
import frc.lib.device.CougarDoubleSolenoid;
import frc.lib.device.DeviceFactory;
import frc.lib.device.LimitSwitch;
import frc.lib.device.MotorController;
import frc.lib.device.PowerDistributor;


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
  public AnalogAccelerometer makeBuiltinAccelerometer(int channel) {
    return new AnalogAccelerometer(channel);
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
      String name, int channel, Type encoderType) {
    return CougarSparkMax.makeBrushless(name, channel, encoderType);
  }

  /**
   * Returns a Brushed CanSparkMax instance.
   */
  @Override
  public AdvancedMotorController makeBrushedCanSparkMax(
      String name, int channel, Type encoderType) {
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

  @Override
  public AdvancedMotorController makeBrushedCanSparkMax(String name, int channel,
      SparkMaxRelativeEncoder.Type encoderType) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'makeBrushedCanSparkMax'");
  }

  @Override
  public AdvancedMotorController makeBrushlessCanSparkMax(String name, int channel,
      SparkMaxRelativeEncoder.Type encoderType) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'makeBrushlessCanSparkMax'");
  }
}
