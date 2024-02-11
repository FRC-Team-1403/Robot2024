package team1403.robot.targeting;

/**
 * A point in 2D space.
 */
public final class Point {
  public Point(int x, int y) {
    x_ = x;
    y_ = y;
  }
  public int x() { return x_; }
  public int y() { return y_; }

  /**
   * Compute the distance to another point.
   */
  public double distance(Point other) {
    double xSqr = (other.x_ - x_) * (other.x_ - x_);
    double ySqr = (other.y_ - y_) * (other.y_ - y_);
    return Math.sqrt(xSqr + ySqr);
  }

  /**
   * Determine if this point is near another.
   *
   * @param p The point to check against.
   * @param threshold The minimum distance to be considered "near".
   */
  public boolean isNear(Point p, double threshold) {
    return distance(p) <= threshold;
  }

  public boolean equals(Object o) {
    if (o == null || o.getClass() != Point.class) {
	return false;
    }

    Point p = (Point)o;
    return p.x_ == x_ && p.y_ == y_;
  }

  public String toString() {
    return "(" + x_ + "," + y_ + ")";
  }

  private final int x_;
  private final int y_;
}
