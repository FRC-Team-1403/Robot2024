package team1403.lib.device.wpi;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import team1403.lib.device.GyroscopeDevice;

/**
 * The NavX implementation of the gyroscope device. Assumes the NavX is
 * connected to the MXP port.
 */
public class NavxAhrs extends AHRS implements GyroscopeDevice {
  private final String m_name;

  public NavxAhrs(String name) {
    super(edu.wpi.first.wpilibj.SerialPort.Port.kUSB);
    this.m_name = name;
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

  @Override
  public Rotation2d getRotation2d() {
      // TODO Auto-generated method stub
      return super.getRotation2d();
  }
}