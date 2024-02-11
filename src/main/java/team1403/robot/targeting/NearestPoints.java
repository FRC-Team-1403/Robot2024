package team1403.robot.targeting;

import java.util.ArrayList;
import java.util.List;

/**
 * Two points and each's distance to an implicit third point.
 * 
 * Maybe we need 3-4 points where we are on or in middle of all 3-4 or on
 * an edge. Otherwise we could have something like this:
 *      A
 *        C
 *      B
 * where A and B only have 1 different dimension of data to interpolate.
 *
 * 3 sounds better but 4 sounds easier to determine. Need to define all 4
 * farthest points anyway.
 *
 * Because we need to find reference points in the relative quadrants,
 * it means point X would be outside in the following scenario:
 *           a
 *       b    X c
 *         d
 * because there is no point to the top-right of it. This limitation
 * makes for a simpler solution. Too avoid the consequences calibrate
 * the outer corners of viable points you want to be in range then
 * add in additional points as more accuracy is needed in certain
 * zones where the interpolation would have too much error.
 */
public class NearestPoints {

   /**
    * Denotes relative positioning of a point from a reference.
    *
    * This would be TL.
    *     P
    *        Ref
    *
    * This would be both TR and BR so is MULTI
    *     Ref   P
    */
   static public enum Quadrant {
     BL(0),  // Bottom Left 
     BR(1),  // Bottom Right
     TL(2),  // Top Left
     TR(3),  // Top Right
     MULTI(-1); // Used for determineQuadrant result only

     /**
      * Returns integer value of quadrant so it can be used as an index.
      */
     public int value() { return value_; }

     private Quadrant(int i) {
       value_ = i;
     }
     private final int value_; 
   }

   /**
    * Find the relative quadrant of point relative to another.
    *
    * @param p The point whose relative quadrant we want.
    * @param relativeTo The reference point we are comparing against.
    *
    * @see determineMultiQuadrants
    */
   static public Quadrant determineQuadrant(Point p, Point relativeTo) {
     int dx = p.x() - relativeTo.x();
     int dy = p.y() - relativeTo.y();

     if (dx * dy == 0) {
	// At least one is 0 so we are on the same x or y.
       return Quadrant.MULTI;
     }

     if (dx > 0) {
       return dy > 0 ? Quadrant.TR : Quadrant.BR;
     }
     return dy > 0 ? Quadrant.TL : Quadrant.BL;
   }

   /**
    * Returns a list of relative quadrants from one point to another.
    *
    * The list would be of size 1 unless the points are aligned on
    * one or more axis in whcih case there could be 2 or 4 results
    * (4 being if the points are the same).
    *
    * @see determineQuadrant
    */
   static public List<Quadrant> determineMultiQuadrants(
     Point p, Point relativeTo) {
     var results = new ArrayList<Quadrant>();

     int dx = p.x() - relativeTo.x();
     int dy = p.y() - relativeTo.y();

     if (dx <= 0) {
       if (dy >= 0) {
         results.add(Quadrant.TL);
       }       
       if (dy <= 0) {
         results.add(Quadrant.BL);
       }
     }
     if (dx >= 0) {
       if (dy >= 0) {
         results.add(Quadrant.TR);
       }       
       if (dy <= 0) {
         results.add(Quadrant.BR);
       }
     }
      
     return results;
   }

   /**
    * Return the nearest surrounding points in list to the given point.
    *
    * @param p The desired point to compare against.
    * @param points The list of points to find among.
    *
    * @result Check isOutOfBounds to see if p is not within points.
    * @see isOutOfBounds
    */
   public static NearestPoints findNearestPoints(
     Point p,
     List<Point> points) {
     // Initially result is the worst possible point in each quadrant.
     // (biggest integer position) so anything we come across in the
     // quadrant (if anything at all) will be better.
     NearestPoints result = new NearestPoints();

     // Iterate over the reference points.
     var it = points.listIterator();
     while (it.hasNext()) {
       Point ref = it.next(); 
       double dRef = p.distance(ref);
       var quadrant = determineQuadrant(ref, p);
       if (quadrant != Quadrant.MULTI) {
	  // Keep this point if it is closer than prev best in quadrant.
         result.maybeUpdatePoint(ref, dRef, quadrant);
         continue;
       }

       // Same as above but consider the reference point for each of
       // multiple quadrants.
       var list = determineMultiQuadrants(ref, p);
       var listIt = list.listIterator();
       while (listIt.hasNext()) {
         result.maybeUpdatePoint(ref, dRef, listIt.next());
       }
     }

     // Note we may have never updated one or more quadrants.
     // If that is the case, then result.isOutOfBounds() otherwise not.
     return result;
   }

   /**
    * Returns the list of nearest boundary points indexed by Quadrant.
    *
    * Do not modify this array.
    */
   public Point[] points() {
     return points_;
   }

   /**
    * Returns the list of nearest boundary point distances indexed by Quadrant.
    *
    * Do not modify this array.
    */
   public double[] distances() {
     return distances_;
   }

   /**
    * If point is closer that previous nearest point for quadrant then use it.
    *
    * This assumes the reference point is same as existing nearest points.
    *
    * @param p The proposed point.
    * @param d The distance of the proposed point to the reference.
    * @param q The relative quadrant of p to the reference.
    *
    * @return true if the point is now the nearest for the quadrant.
    */
   public boolean maybeUpdatePoint(Point p, double d, Quadrant q) {
      int i = q.value();
      if (d < distances_[i]) {
          points_[i] = p;
          distances_[i] = d;
          return true;
      }
      return false;
   }

   /**
    * Determine if any of our points are out of bounds.
    */
   public boolean isOutOfBounds() {
      // Out of bounds means the original value never changed.
      // Since the field bounds are much smaller than the max int value
      // we can just add all the X and Y values and look at the final
      // magnitude. If one was out of bounds then it will be huge.
      //
      // The bottom values are -MIN so we will negate them.
      //
      // If one of these was MAX_VALUE then we're going to overflow int
      // so we'll cast to long when adding.
      long sumX = -(long)points_[Quadrant.BL.value()].x()
                  + (long)points_[Quadrant.BR.value()].x()
                  + -(long)points_[Quadrant.TL.value()].x()
                  + (long)points_[Quadrant.TR.value()].x();
      long sumY = -(long)points_[Quadrant.BL.value()].y()
                  + -(long)points_[Quadrant.BR.value()].y()
                  + (long)points_[Quadrant.TL.value()].y()
                  + (long)points_[Quadrant.TR.value()].y();
      return sumX + sumY > (long)Integer.MAX_VALUE;
   }

   // Initially the nearest points are all out of bounds.
   // This is package private so we can test it.
   Point[] points_ = {
      new Point(Integer.MIN_VALUE, Integer.MIN_VALUE), // BL
      new Point(Integer.MAX_VALUE, Integer.MIN_VALUE),  // BR
      new Point(Integer.MIN_VALUE, Integer.MAX_VALUE),  // TL
      new Point(Integer.MAX_VALUE, Integer.MAX_VALUE),   // TR
   };

   // This is package private so we can test it.
   double[] distances_ = {
       Integer.MAX_VALUE, Integer.MAX_VALUE,
       Integer.MAX_VALUE, Integer.MAX_VALUE
   };
}
