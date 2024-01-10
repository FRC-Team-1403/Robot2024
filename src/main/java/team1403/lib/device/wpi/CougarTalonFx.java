package team1403.lib.device.wpi;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import team1403.lib.device.AdvancedMotorController;
import team1403.lib.device.CurrentSensor;
import team1403.lib.device.Encoder;
import team1403.lib.device.NoSuchDeviceError;

/**
 * Device implementation for a base TalonFX motor controller.
 */
public class CougarTalonFx implements AdvancedMotorController {
  /**
   * Constructor.
   *
   * @param name of the mptor
   * @param deviceNumber the port the motor is plugged into
   * @param controlMode the control mode of the motor
   */
  public CougarTalonFx(String name, int deviceNumber) {
    m_motor = new TalonFX(deviceNumber);
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
    return m_motor;
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public void follow(AdvancedMotorController source) {
    //m_motor.follow((IMotorController)source);
    System.err.println("[follow] not yet supported on TalonFx!");
  }

  @Override
  public final void setVoltageCompensation(double voltage) {
    //fixme
    System.err.println("[setVoltageCompensation] not yet supported on TalonFx");
  }

  @Override
  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  //TODO: units are unkown
  @Override
  public void setPosition(double position) {
    StatusCode code = m_motor.setPosition(position);
    if(code.isError())
      System.out.println(code.getDescription());
  }

  @Override
  public void setInverted(boolean isInverted) {
    m_motor.setInverted(isInverted);
  }

  @Override 
  public void setPidGains(double p, double i, double d) {
    System.err.println("[setPidGains] not yet supported on TalonFx!");
  }

  @Override
  public void setIdleMode(CougarIdleMode mode) {
    if (mode == CougarIdleMode.BRAKE) {
      m_motor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      m_motor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void setRampRate(double rate) {
    System.err.println("[setRampRate] not yet supported on TalonFx!");
  }

  @Override
  public void stopMotor() {
    m_motor.stopMotor();
  }

  @Override
  public void setAmpLimit(double amps) {
    System.err.println("[setAmpLimit] not yet supported on TalonFx!");
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
  public boolean getInverted() {
    return m_motor.getInverted(); 
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
     
      return m_motor.getPosition().getValueAsDouble() * m_positionConversionFactor;
    }

    @Override
    public final double getVelocityValue() {
      return m_motor.getVelocity().getValueAsDouble() * m_velocityConversionFactor;
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
      m_motor.setPosition(position);
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
      return m_motor.getStatorCurrent().getValueAsDouble();
    }

    private final String m_sensorName;
  }

  private final EmbeddedEncoder m_encoder;
  private final EmbeddedCurrentSensor m_currentSensor;
  private final String m_name;
  private final TalonFX m_motor;

  private double m_positionConversionFactor = 1;
  private double m_velocityConversionFactor = 1;
}
