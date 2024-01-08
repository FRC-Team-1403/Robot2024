package frc.lib.device.wpi;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.lib.device.AdvancedMotorController;
import frc.lib.device.CurrentSensor;
import frc.lib.device.Encoder;
import frc.lib.device.NoSuchDeviceError;

/**
 * Device implementation for a base TalonFX motor controller.
 */
public class CougarTalonFx extends TalonFX implements AdvancedMotorController {
  /**
   * Constructor.
   *
   * @param name of the mptor
   * @param deviceNumber the port the motor is plugged into
   * @param controlMode the control mode of the motor
   */
  public CougarTalonFx(String name, int deviceNumber) {
    super(deviceNumber);
    m_name = name;
    m_encoder = new EmbeddedEncoder(name + ".Encoder");
    m_currentSensor = new EmbeddedCurrentSensor(name + ".CurrentSensor");
  }

  /**
   * Return the TalonFX API so we can do something specific.
   *
   * @return The underlying {@code TalonFX} instance.
   */
  public final TalonFX geTalonFxApi() {
    return this;
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public void follow(AdvancedMotorController source) {
    super.follow((TalonFX) source);
  }

  @Override
  public final void setVoltageCompensation(double voltage) {
    super.configVoltageCompSaturation(voltage);
  }

  @Override
  public void setSpeed(double speed) {
    set(TalonFXControlMode.PercentOutput, speed);
  }

  //TODO: units are unkown
  @Override
  public void setPosition(double position) {
    set(TalonFXControlMode.Position, position);
  }

  @Override
  public void setInverted(boolean isInverted) {
    super.setInverted(isInverted);
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
      super.setNeutralMode(NeutralMode.Coast);
    }
  }

  @Override
  public void setRampRate(double rate) {
    super.configClosedloopRamp(rate);
  }

  @Override
  public void stopMotor() {
    set(TalonFXControlMode.Velocity, 0);
  }

  @Override
  public void setAmpLimit(double amps) {
    super.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, amps, 0, 0));
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

  private double m_positionConversionFactor = 1;
  private double m_velocityConversionFactor = 1;
}
