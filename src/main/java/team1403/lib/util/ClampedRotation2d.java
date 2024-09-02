package team1403.lib.util;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.Radians;

//Rotation2d class but clamps angles to [-pi, pi]
public class ClampedRotation2d extends Rotation2d {


  /** Constructs a Rotation2d with a default angle of 0 degrees. */
  public ClampedRotation2d() {
    super();
  }

  /**
   * Constructs a Rotation2d with the given radian value.
   *
   * @param value The value of the angle in radians.
   */
  @JsonCreator
  public ClampedRotation2d(@JsonProperty(required = true, value = "radians") double value) {
    super(MathUtil.angleModulus(value));
  }

  /**
   * Constructs a Rotation2d with the given angle.
   *
   * @param angle The angle of the rotation.
   */
  public ClampedRotation2d(Measure<Angle> angle) {
    this(angle.in(Radians));
  }

  /**
   * Constructs a Rotation2d with the given x and y (cosine and sine) components.
   *
   * @param x The x component or cosine of the rotation.
   * @param y The y component or sine of the rotation.
   */
  public ClampedRotation2d(double x, double y) {
    super(x, y);
  }

  /**
   * Constructs and returns a Rotation2d with the given radian value.
   *
   * @param radians The value of the angle in radians.
   * @return The rotation object with the desired angle value.
   */
  public static Rotation2d fromRadians(double radians) {
    return new ClampedRotation2d(radians);
  }

  /**
   * Constructs and returns a Rotation2d with the given degree value.
   *
   * @param degrees The value of the angle in degrees.
   * @return The rotation object with the desired angle value.
   */
  public static Rotation2d fromDegrees(double degrees) {
    return new ClampedRotation2d(Math.toRadians(degrees));
  }

  /**
   * Constructs and returns a Rotation2d with the given number of rotations.
   *
   * @param rotations The value of the angle in rotations.
   * @return The rotation object with the desired angle value.
   */
  public static Rotation2d fromRotations(double rotations) {
    return new ClampedRotation2d(Units.rotationsToRadians(rotations));
  }
}
