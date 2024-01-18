package team1403.lib.device.wpi;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import team1403.lib.device.AdvancedMotorController;
import team1403.lib.device.CurrentSensor;
import team1403.lib.device.Encoder;
import team1403.lib.device.NoSuchDeviceError;

/**
 * Device implementation for a base CANSparkMax motor controller.
 */
public final class CougarSparkMax extends CANSparkMax implements AdvancedMotorController {
  /**
   * Method for creating brushless CANSparkMax.
   *
   * @param name        The name name for the device.
   * @param channel     The CAN channel the motor is on.
   * @param encoderType the type of encoder attached
   * @return Brushless CANSparkMax
   */
  public static CougarSparkMax makeBrushless(String name, int channel,
      SparkRelativeEncoder.Type encoderType) {
    return new CougarSparkMax(name, channel, MotorType.kBrushless, encoderType);
  }

  /**
   * Method for creating brushed CANSparkMax.
   *
   * @param name        The name name for the device.
   * @param channel     The CAN channel the motor is on.
   * @param encoderType the type of encoder attached
   * @return Brushed CANSparkMax
   */
  public static CougarSparkMax makeBrushed(String name, int channel,
      SparkRelativeEncoder.Type encoderType) {
    return new CougarSparkMax(name, channel, MotorType.kBrushed, encoderType);
  }

  /**
   * Constructor.
   *
   * @param name        The name name for the device.
   * @param channel     The CAN channel the motor is on.
   * @param motorType   The type of motor connected
   * @param encoderType the type of encoder attached
   */
  private CougarSparkMax(String name, int channel, MotorType motorType,
      SparkRelativeEncoder.Type encoderType) {
    super(channel, motorType);
    m_name = name;
    if (encoderType == SparkRelativeEncoder.Type.kNoSensor) {
      m_encoder = null;
    } else if (encoderType == SparkRelativeEncoder.Type.kHallSensor) {
      m_encoder = new EmbeddedEncoder(name + ".Encoder", getEncoder(encoderType, 42));
    } else {
      m_encoder = new EmbeddedEncoder(name + ".Encoder", getEncoder(encoderType, 4096));
    }
    m_currentSensor = new EmbeddedCurrentSensor(name + ".CurrentSensor");
  }

  /**
   * Return the CANSparkMax API so we can do something specific.
   *
   * @return The underlying {@code CANSparkMax} instance.
   */
  public final CANSparkMax getCanSparkMaxApi() {
    return this;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  /**
   * Follow another CANSParkMax motor.
   *
   * @param source Must be a com.revrobotics.CANSparkMax
   *
   * @throws ClassCastException if motor is not compatible.
   */
  @Override
  public void follow(AdvancedMotorController source) {
    follow((CANSparkMax) source); // Will throw an exception if source is not compatible.
  }

  @Override
  public final void setVoltageCompensation(double voltage) {
    super.enableVoltageCompensation(voltage);
  }

  @Override
  public final void setSpeed(double speed) {
    super.set(speed);
  }

  @Override
  public void setPosition(double position) {
  }

  @Override
  public void setPidGains(double p, double i, double d) {
    super.getPIDController().setP(p);
    super.getPIDController().setI(i);
    super.getPIDController().setD(d);
  }

  @Override
  public void setIdleMode(CougarIdleMode mode) {
    if (mode == CougarIdleMode.BRAKE) {
      super.setIdleMode(IdleMode.kBrake);
    } else {
      super.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public final void setRampRate(double ramp) {
    setClosedLoopRampRate(ramp);
  }

  @Override
  public final void stopMotor() {
    super.stopMotor();
  }

  @Override
  public void setAmpLimit(double amps) {
    super.setSmartCurrentLimit((int)amps);
  }

  @Override
  public boolean hasEmbeddedEncoder() {
    return m_encoder != null;
  }

  @Override
  public Encoder getEmbeddedEncoder() {
    if (hasEmbeddedEncoder()) {
      return m_encoder;
    }
    throw new NoSuchDeviceError("No Encoder with " + m_name);
  }

  @Override
  public boolean hasEmbeddedCurrentSensor() {
    return true;
  }

  @Override
  public CurrentSensor getEmbeddedCurrentSensor() {
    return m_currentSensor;
  }

  /**
   * Implements the interface to the embedded encoder.
   */

  private class EmbeddedEncoder implements Encoder {
    /**
     * Constructor.
     *
     * @param name        The name of encoder
     * @param encoderType the type of encoder used
     */
    public EmbeddedEncoder(String name, RelativeEncoder encoder) {
      m_encoderName = name;
      m_encoder = encoder;
    }

    @Override
    public final String getName() {
      return m_encoderName;
    }

    @Override
    public final double ticksPerRevolution() {
      return m_encoder.getCountsPerRevolution();
    }

    @Override
    public final double getPositionValue() {
      return m_encoder.getPosition();
    }

    @Override
    public final double getVelocityValue() {
      return m_encoder.getVelocity();
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
      m_encoder.setPositionConversionFactor(conversionFactor);
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
      m_encoder.setVelocityConversionFactor(conversionFactor);
    }

    @Override
    public void setPositionOffset(double position) {
      m_encoder.setPosition(position);
    }
    
    private final String m_encoderName;
    private final RelativeEncoder m_encoder;
  }

  /**
   * Implements the interface to the embedded current sensor.
   *
   * <p>This is not a static class so instances share the
   * CougarSparkMax instance state.
   */
  private class EmbeddedCurrentSensor implements CurrentSensor {
    /**
     * Constructor.
     */
    public EmbeddedCurrentSensor(String name) {
      m_sensorName = name;
    }

    @Override
    public final String getName() {
      return m_sensorName;
    }

    @Override
    public final double getAmps() {
      return getOutputCurrent();
    }

    private final String m_sensorName;
  }

  private final EmbeddedEncoder m_encoder;
  private final EmbeddedCurrentSensor m_currentSensor;
  private final String m_name;
}
