package team1403.lib.device.wpi;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import team1403.lib.device.GyroscopeDevice;
import team1403.lib.util.ClampedRotation2d;

/**
 * The NavX implementation of the gyroscope device. Assumes the NavX is
 * connected to the MXP port.
 */
public class NavxAhrs extends AHRS implements GyroscopeDevice {
  private final String m_name;

  public NavxAhrs(String name, SerialPort.Port port)
  {
    super(port);
    this.m_name = name;
  }

  public NavxAhrs(String name) {
    this(name, SerialPort.Port.kMXP);
  } 

  @Override
  public String getName() { 
    return m_name;
  }

  @Override
  public double getRawAngle() {
    return getAngle() - getAngleAdjustment();
  }

  @Override
  public double getAngularVelocity() {
    return getRate();
  }

  @Override
  public void setAngleOffset(double angleOffset) {
    setAngleAdjustment(angleOffset);
  }

  @Override
  public double getAngleOffset() {
    return getAngleAdjustment();
  }

  public Rotation2d getRawRotation2d()
  {
    return super.getRotation2d();
  }

  private Rotation2d get180to180Rotation2d(){
    return ClampedRotation2d.fromDegrees(-getAngle());
  }

  @Override
  public Rotation2d getRotation2d() {
    return get180to180Rotation2d();
  }
}