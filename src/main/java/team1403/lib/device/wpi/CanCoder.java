package team1403.lib.device.wpi;

import com.ctre.phoenix6.hardware.CANcoder;

import team1403.lib.device.Encoder;


/**
 * Implements the encoder.
 */
public class CanCoder extends CANcoder implements Encoder {

  private final double m_ticksPerRevolution;
  private double m_positionConversionFactor = 1.0;
  private double m_velocityConversionFactor = 1.0;
  private final String m_name;

  /**
   * defines the name, port number, ticks per revolution.
   *
   * @param name The name of the instance
   * @param portNumber The number of the port for the CanCoder
   * @param ticksPerRevolution The ticks per revolution is fixed.
   */
  public CanCoder(String name, int portNumber) {
    super(portNumber);
    m_name = name;
    //source: https://store.ctr-electronics.com/content/user-manual/CANCoder%20User%27s%20Guide.pdf
    m_ticksPerRevolution = 4096;
  }

  @Override
  public double getPositionValue() {
    return super.getPosition().getValueAsDouble() * m_positionConversionFactor;
  }

  @Override
  public double getVelocityValue() {
    return super.getVelocity().getValueAsDouble() * m_velocityConversionFactor;
  }

  @Override
  public double ticksPerRevolution() {
    return m_ticksPerRevolution;
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
  public String getName() {
    return m_name;
  }

  @Override
  public void setPositionOffset(double position) {
    setPosition(position);
  }
}