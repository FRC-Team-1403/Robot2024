package team1403.robot.targeting;

import java.util.ArrayList;
import java.util.HashMap;


/**
 * Computes the TargetingParams for a given point on the field.
 *
 * Usage:
 *    var calculator = new TargetingCalculator();
 *    calculator.addReferencePoint(ref1, params1);
 *    ...
 *    calculator.addReferencePoint(refN, paramsN);
 *
 *    Point arbitraryPoint = new Point(x, y);
 *    TargetingParams solution = calculator.computeParams(arbitraryPoint);
 *
 * Where params1..paramsN are the calibrated targeting solutions for
 * reference points ref1..refN.
 *
 * Design:
 *   The calculator contains a number of arbitrary refererence points
 *   within an arbitrarily sized field grid (within NearestPoints constraints).
 *   Each of these points has registered calibrated TargetingParams. The
 *   calculatorr will then use these to `computeParrams` for a given
 *   arbitrary point on the field.
 *
 *   To do this it will use NearestPoints to find the nearest points to
 *   to desired point from among the previously added reference points.
 *   If the provided point is reported as out of bounds then the
 *   `kOutOfBouonds` solution is returned. Otherwise the TargetingParams
 *   recommendation is derived from the corresponding TargetingParams to
 *   the nearest reference points.
 *
 *   The derived parameters for each of the (angle, speed, heading) are
 *   individually computed using `interpolate*` methods where their
 *   algorithms can be tuned individually as appropriate.
 *
 * @see NearestPoints
 */
public class TargetingCalculator {
  /**
   * Construct new calculator.
   *
   * We assume that the field origin is at one corner.
   *
   * @param maxX The largest X grid point.
   * @param maxY The largest Y grid point.
   */
  public TargetingCalculator(int maxX, int maxY) {
    maxFieldX_ = maxX;
  }

  public void addReferencePoint(Point point, TargetingParams params) {
     int id = normalizedId(point);

     if (normalizedIdToPointIndex_.containsKey(id)) {
       // Ths system.err is bad here, but indicates a bug that should
       // be addressed immediately so should never happen in a match.
       System.err.println("Unexpected duplicate reference point " + point);
       return;
     }
     normalizedIdToPointIndex_.put(id, points_.size());
     points_.add(point);     
     params_.add(params);
  }

  /**
   * Computes targeting parameters for the given point.
   * This requires us to be "inside" the points
   */
  public TargetingParams computeParams(Point point) {
     var nearest = NearestPoints.findNearestPoints(point, points_);
     if (nearest.isOutOfBounds()) {
        System.err.println("Point " + point + " is out of bounds.");
        return TargetingParams.kOutOfBounds;
     }

     var points = nearest.points();
     var distances = nearest.distances();
     var params = new TargetingParams[4];
     params[0] = pointToParams(points[0]);
     params[1] = pointToParams(points[1]);
     params[2] = pointToParams(points[2]);
     params[3] = pointToParams(points[3]);

     return new TargetingParams(
        interpolateAngle(params, distances),
        interpolateSpeed(params, distances),
        interpolateHeading(params, distances));
  }

  /**
   * Protected for testing purposes
   *
   * @param params The calibrated parameters indexed by relative quadrant.
   * @param distances The distances from calibrated points for params.
   */
  protected double interpolateAngle(
      TargetingParams[] params, double[] distances) {
    return Double.NaN;
  }
  
  protected double interpolateSpeed(
      TargetingParams[] params, double[] distances) {
    return Double.NaN;
  }
  
  protected double interpolateHeading(
      TargetingParams[] params, double[] distances) {
    return Double.NaN;
  }
  
  /**
   *  Returns a unique integer for each Point on the field.
   */
  private int normalizedId(Point point) {
    return point.y() * maxFieldX_ + point.x();
  }

  /**
   * Return the targeting parameters for the given reference point.
   *
   * Assumes p is among points_.
   */
  private TargetingParams pointToParams(Point p) {
     int i = normalizedId(p);
     return params_.get(normalizedIdToPointIndex_.get(i));
  }


  // The list of reference points in arbitrary order.
  private ArrayList<Point> points_ = new ArrayList<Point>();

  // The list of reference point targeting params in order of points.
  private ArrayList<TargetingParams> params_
      = new ArrayList<TargetingParams>();

  // Maps normalizedId of a reference point to its List index.
  // This is so we can keep our simple point types and operations on them.
  // Otherwise we'd need to manage a more complex type, which would be ok but
  // we chose not to.
  private HashMap<Integer, Integer> normalizedIdToPointIndex_
      = new HashMap<Integer, Integer>();

  private int maxFieldX_;
}
