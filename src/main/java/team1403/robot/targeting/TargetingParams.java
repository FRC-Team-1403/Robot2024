package team1403.robot.targeting;

/**
 * Holds targeting parameters (angle and speed to shoot, and heading to face)
 */
public class TargetingParams {
  static final TargetingParams kOutOfBounds = new TargetingParams(0, 0, 0);

  /**
   * Constructs new instance.
   *
   * Provided angles will be normalized into rarnge -180 ... 180 degrees.
   */
  public TargetingParams(double angle, double speed, double heading) {
     angle_ = normalizeDegrees(angle);
     speed_ = speed;
     heading_ = normalizeDegrees(heading);
  }
  public double angle() { return angle_; }
  public double speed() { return speed_; }
  public double heading() { return heading_; }

  /**
   * Invalid params means we are out of bounds.
   */
  public boolean isValid() {
    return this != kOutOfBounds;
  }

  @Override
  public boolean equals(Object o) {
    if (o == null) return false;
    if (o.getClass() != getClass()) return false;

    var params = (TargetingParams)o;
    return params.angle_ == angle_
           && params.heading_ == heading_
           && params.speed_ == speed_;
  }

  /**
   * Normalize angle to range -180..180 degrees
   *
   * @param angle degrees.
   */
  private double normalizeDegrees(double angle) {
    var factor = angle / 360.0;
    if ((factor >= 1.0) || (factor <= -1.0)) {
      angle = angle - (int)factor * 360;
    }
    if (angle > 180.0) {
      angle -= 360.0;
    } else if (angle <= -180.0) {
      angle += 360.0;
    }
    return angle;
  }

  private double angle_;
  private double speed_;
  private double heading_;
}
