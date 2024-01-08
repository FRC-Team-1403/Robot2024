package frc.lib.device.wpi;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.lib.device.AdvancedMotorController;
import frc.lib.device.CurrentSensor;
import frc.lib.device.Encoder;

/**
 * Device implementation for a WPI_TalonSRX motor controller.
 */
public class TalonSrx extends WPI_TalonSRX
                      implements AdvancedMotorController {
  /**
   * Constructor.
   *
   * @param name The name name for the device.
   * @param channel The CAN channel the motor is on.
   */
  public TalonSrx(String name, int channel) {
    super(channel);
    m_name = name;
    m_encoder = new EmbeddedEncoder(name + ".Encoder");
    m_currentSensor = new EmbeddedCurrentSensor(name + ".CurrentSensor");
  }

  /**
   * Return the WPI_TalonSRX API so we can do something specific.
   *
   * @return The underlying {@code WPI_TalonSRX} instance.
   */
  public final WPI_TalonSRX getTalonSrxApi() {
    return this;
  }

  @Override
  public final String getName() {
    return m_name;
  }

  /**
   * Follow another TalonSrx motor.
   *
   * @param source Must be a com.ctre.phoenix.motorcontrol.IMotorController
   *               (e.g. another TalonSrx or VictorSpx)
   *
   * @throws ClassCastException if motor is not compatible.
   */
  @Override
  public void follow(AdvancedMotorController source) {
    super.follow((IMotorController)source);  // Will throw an exception if source is not compatible.
  }

  @Override
  public final void setVoltageCompensation(double voltage) {
    super.configVoltageCompSaturation(voltage);
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
    super.config_kP(0, p);
    super.config_kD(0, d);
    super.config_kI(0, i);
  }

  @Override
  public void setIdleMode(CougarIdleMode mode) {
    if (mode == CougarIdleMode.BRAKE) {
      super.setNeutralMode(NeutralMode.Brake);
    } else {
      super.setNeutralMode(NeutralMode.Brake);
    }
  }

  @Override
  public void setRampRate(double rate) {
    configClosedloopRamp(rate);
  }

  @Override
  public void setAmpLimit(double amps) {
    super.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, amps, 0, 0));
  }

  @Override
  public boolean hasEmbeddedEncoder() {
    return true;
  }

  @Override
  public Encoder getEmbeddedEncoder() {
    return m_encoder;
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
     */
    public EmbeddedEncoder(String name) {
      m_encoderName = name;
    }

    @Override
    public final String getName() {
      return m_encoderName;
    }

    @Override
    public final double ticksPerRevolution() {
      return 4096;
    }

    @Override
    public final double getPositionValue() {
      return getSelectedSensorPosition() * m_positionConversionFactor;
    }

    @Override
    public final double getVelocityValue() {
      return getSelectedSensorVelocity() * m_velocityConversionFactor;
    }

    @Override
    public void setPositionConversionFactor(double conversionFactor) {
      m_positionConversionFactor = conversionFactor;
    }

    @Override
    public void setVelocityConversionFactor(double conversionFactor) {
      m_velocityConversionFactor = conversionFactor;
    }

    @Override
    public void setPositionOffset(double position) {
      setSelectedSensorPosition(position);
    }

    private final String m_encoderName;
  }

  /**
   * Implements the interface to the embedded current sensor.
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
      return getStatorCurrent();
    }

    private final String m_sensorName;
  }

  private final EmbeddedEncoder m_encoder;
  private final EmbeddedCurrentSensor m_currentSensor;
  private final String m_name;

  private double m_positionConversionFactor = 1.0;
  private double m_velocityConversionFactor = 1.0;
}
