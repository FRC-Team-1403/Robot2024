package team1403.lib.util;

import java.util.Objects;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Radians;

/*
 * A Rotation2d class that is mutable (to a very small extent)
 * Danger!!!! Make sure the class/classes you use this on create a copy of it!
 * Don't let any other classes use this class directly, cast to Rotation2d first!!
 */
public class Rotation2dM extends Rotation2d {

  private double m_value;
  private double m_sin;
  private double m_cos;


  /** Constructs a Rotation2d with a default angle of 0 degrees. */
  public Rotation2dM() {
    m_value = 0.0;
    m_cos = 1.0;
    m_sin = 0.0;
  }

  /** 
   * Sets the value of the Rotation2dM in radians 
   * 
   * @param value The value of the angle in radians.
   */
  public Rotation2dM set(double value) {
    m_value = value;
    m_cos = Math.cos(value);
    m_sin = Math.sin(value);
    return this;
  }

  /**
   * Constructs a Rotation2d with the given radian value.
   *
   * @param value The value of the angle in radians.
   */
  @JsonCreator
  public Rotation2dM(@JsonProperty(required = true, value = "radians") double value) {
    set(value);
  }

  /**
   * Constructs a Rotation2d with the given x and y (cosine and sine) components.
   *
   * @param x The x component or cosine of the rotation.
   * @param y The y component or sine of the rotation.
   */
  public Rotation2dM(double x, double y) {
    double magnitude = Math.hypot(x, y);
    if (magnitude > 1e-6) {
      m_sin = y / magnitude;
      m_cos = x / magnitude;
    } else {
      m_sin = 0.0;
      m_cos = 1.0;
    }
    m_value = Math.atan2(m_sin, m_cos);
  }

  /**
   * Constructs a Rotation2d with the given angle.
   *
   * @param angle The angle of the rotation.
   */
  public Rotation2dM(Measure<Angle> angle) {
    this(angle.in(Radians));
  }

  /**
   * Constructs and returns a Rotation2d with the given radian value.
   *
   * @param radians The value of the angle in radians.
   * @return The rotation object with the desired angle value.
   */
  public static Rotation2dM fromRadians(double radians) {
    return new Rotation2dM(radians);
  }

  /**
   * Constructs and returns a Rotation2d with the given degree value.
   *
   * @param degrees The value of the angle in degrees.
   * @return The rotation object with the desired angle value.
   */
  public static Rotation2dM fromDegrees(double degrees) {
    return new Rotation2dM(Math.toRadians(degrees));
  }

  /**
   * Constructs and returns a Rotation2d with the given number of rotations.
   *
   * @param rotations The value of the angle in rotations.
   * @return The rotation object with the desired angle value.
   */
  public static Rotation2dM fromRotations(double rotations) {
    return new Rotation2dM(Units.rotationsToRadians(rotations));
  }

  /**
   * Adds two rotations together, with the result being bounded between -pi and pi.
   *
   * <p>For example, <code>Rotation2d.fromDegrees(30).plus(Rotation2d.fromDegrees(60))</code> equals
   * <code>Rotation2d(Math.PI/2.0)</code>
   *
   * @param other The rotation to add.
   * @return The sum of the two rotations.
   */
  public Rotation2d plus(Rotation2d other) {
    return rotateBy(other);
  }

  /**
   * Subtracts the new rotation from the current rotation and returns the new rotation.
   *
   * <p>For example, <code>Rotation2d.fromDegrees(10).minus(Rotation2d.fromDegrees(100))</code>
   * equals <code>Rotation2d(-Math.PI/2.0)</code>
   *
   * @param other The rotation to subtract.
   * @return The difference between the two rotations.
   */
  public Rotation2d minus(Rotation2d other) {
    return rotateBy(other.unaryMinus());
  }

  /**
   * Takes the inverse of the current rotation. This is simply the negative of the current angular
   * value.
   *
   * @return The inverse of the current rotation.
   */
  public Rotation2d unaryMinus() {
    return new Rotation2d(-m_value);
  }

  /**
   * Multiplies the current rotation by a scalar.
   *
   * @param scalar The scalar.
   * @return The new scaled Rotation2d.
   */
  public Rotation2d times(double scalar) {
    return new Rotation2d(m_value * scalar);
  }

  /**
   * Divides the current rotation by a scalar.
   *
   * @param scalar The scalar.
   * @return The new scaled Rotation2d.
   */
  public Rotation2d div(double scalar) {
    return times(1.0 / scalar);
  }

  /**
   * Adds the new rotation to the current rotation using a rotation matrix.
   *
   * <p>The matrix multiplication is as follows:
   *
   * <pre>
   * [cos_new]   [other.cos, -other.sin][cos]
   * [sin_new] = [other.sin,  other.cos][sin]
   * value_new = atan2(sin_new, cos_new)
   * </pre>
   *
   * @param other The rotation to rotate by.
   * @return The new rotated Rotation2d.
   */
  public Rotation2d rotateBy(Rotation2d other) {
    return new Rotation2d(
        m_cos * other.getCos() - m_sin * other.getSin(), m_cos * other.getSin() + m_sin * other.getCos());
  }

  /**
   * Returns the radian value of the Rotation2d.
   *
   * @return The radian value of the Rotation2d.
   * @see edu.wpi.first.math.MathUtil#angleModulus(double) to constrain the angle within (-pi, pi]
   */
  @JsonProperty
  public double getRadians() {
    return m_value;
  }

  /**
   * Returns the degree value of the Rotation2d.
   *
   * @return The degree value of the Rotation2d.
   * @see edu.wpi.first.math.MathUtil#inputModulus(double, double, double) to constrain the angle
   *     within (-180, 180]
   */
  public double getDegrees() {
    return Math.toDegrees(m_value);
  }

  /**
   * Returns the number of rotations of the Rotation2d.
   *
   * @return The number of rotations of the Rotation2d.
   */
  public double getRotations() {
    return Units.radiansToRotations(m_value);
  }

  /**
   * Returns the cosine of the Rotation2d.
   *
   * @return The cosine of the Rotation2d.
   */
  public double getCos() {
    return m_cos;
  }

  /**
   * Returns the sine of the Rotation2d.
   *
   * @return The sine of the Rotation2d.
   */
  public double getSin() {
    return m_sin;
  }

  /**
   * Returns the tangent of the Rotation2d.
   *
   * @return The tangent of the Rotation2d.
   */
  public double getTan() {
    return m_sin / m_cos;
  }

  @Override
  public String toString() {
    return String.format("Rotation2d(Rads: %.2f, Deg: %.2f)", m_value, Math.toDegrees(m_value));
  }

  /**
   * Checks equality between this Rotation2d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof Rotation2d) {
      var other = (Rotation2d) obj;
      return Math.hypot(m_cos - other.getCos(), m_sin - other.getSin()) < 1E-9;
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_value);
  }

  @Override
  public Rotation2d interpolate(Rotation2d endValue, double t) {
    return plus(endValue.minus(this).times(MathUtil.clamp(t, 0, 1)));
  }
    
}
