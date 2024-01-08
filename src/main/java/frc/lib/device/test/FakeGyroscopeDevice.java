package frc.lib.device.test;

import frc.lib.device.GyroscopeDevice;


/**
* A fake gyroscope that will be at
* rotate at whatever angle it is told to be at.
*/
public class FakeGyroscopeDevice implements GyroscopeDevice {

  private double m_currAngle;
  private double m_currVelocity;
  private double m_angleOffset;

  /**
  * Creates a fake gyroscope.
  * (either FPGA if in simulator or fake if in unit tests)
  */
  public FakeGyroscopeDevice() {
    this.reset();
  }

  /**
  * Sets the current angle the gyroscope is at to the passed in value.
  *
  * @param angle the angle in degrees
  */
  public void setRawAngle(double angle) {
    this.m_currAngle = angle;
  }

  @Override
  public String getName() {
    return m_name;
  }

  @Override
  public void reset() {
    this.m_currAngle = 0;
    this.m_currVelocity = 0;
  }


  @Override
  public double getRawAngle() {
    return this.m_currAngle;
  }

  @Override
  public double getAngularVelocity() {
    return this.m_currVelocity;
  }

  /**
  * Sets the current velocity the gyroscope to the passed in value.
  *
  * @param velocity the velocity (degrees/sec) it is traveling at
  */
  public void setAngularVelocity(double velocity) {
    this.m_currVelocity = velocity;   
  }


  private String m_name;

  @Override
  public double getAngle() {
    return this.m_currAngle;
  }

  @Override
  public void setAngleOffset(double angleOffset) {
    this.m_angleOffset = angleOffset;
    
  }

  @Override
  public double getAngleOffset() {
    return m_angleOffset;
  }
}