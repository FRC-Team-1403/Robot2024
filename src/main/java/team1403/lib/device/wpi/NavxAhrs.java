package team1403.lib.device.wpi;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import team1403.lib.device.GyroscopeDevice;

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

  public Rotation2d get180to180Rotation2d(){
    double a = super.getRotation2d().getDegrees();
    while(a > 180)
    {
      a -= 360;
    }
    while(a < -180)
    {
      a += 360;
    }
    return new Rotation2d(Units.degreesToRadians(a));
  }

  public Rotation2d get0to360Rotation2d(){
    double a = super.getRotation2d().getDegrees();
    while(a > 360)
    {
      a -= 360;
    }
    while(a < -0)
    {
      a += -a + (360 + a);
    }
    return new Rotation2d(Units.degreesToRadians(a));
  }
  
  public Rotation2d get360to360Rotation2d()
  {
    double a = super.getRotation2d().getDegrees();
    while(a > 360)
    {
      a -= 720;
    }
    while(a < -360)
    {
      a += 720;
    }
    return new Rotation2d(Units.degreesToRadians(a));
  }

  @Override
  public Rotation2d getRotation2d() {
      return super.getRotation2d();
  }
}