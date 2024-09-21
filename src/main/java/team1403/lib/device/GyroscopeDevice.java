package team1403.lib.device;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Interface that represents a gyroscope.
 */
public interface GyroscopeDevice extends Sensor {

  /**
   * resets gyro.
   */
  public abstract void reset();

  /**
   * Gets the angle (in degrees) of the gyro.
   *
   * @return the angle the gyroscope is at (relative to where it was when last
   *         reset)
   */
  public abstract double getRawAngle();

  /**
   * Returns the angle of the gyroscope + the offset.
   *
   * @return the raw angle plus the configured offset
   */
  public double getAngle();

  /**
   * Gets the current angular velocity (about vertical axis) of the gyroscope.
   *
   * @return the current angular velocity (int degrees/sec)
   */
  public abstract double getAngularVelocity();

  /**
   * Configures the offset angle set by getAngle.
   */
  public void setAngleOffset(double angleOffset);

  /**
   * Gets the current angle offset.
   *
   * @return the current angle offset (in degrees)
   */
  public double getAngleOffset();

  /**
   * Returns the angle of the gyroscope clamped between -180 and 180 degrees.
   * Note: the math in the getHeading method is used to invert the direction of
   * the gyro for use by wpilib which treats gyros backwards.
   * Gyros are normally clockwise positive. Wpilib wants
   * counter-clockwise positive.
   *
   * @return the clamped angle in degrees
   */
  public default double getHeading() {
    return MathUtil.inputModulus(-getAngle(), -180, 180);
  }

  /**
   * Convenience method to return the heading as a Rotation2d object.
   *
   * @return the heading
   */
  public default Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }
}