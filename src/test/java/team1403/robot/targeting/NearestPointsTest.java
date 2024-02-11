package team1403.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertIterableEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.Arrays;

class NearestPointsTest {
  @Test
  public void testDetermineQuandrant() {
      Point reference = new Point(100, 200);
      assertEquals(
          NearestPoints.Quadrant.BL,
          NearestPoints.determineQuadrant(new Point(50, 150), reference));
      assertEquals(
          NearestPoints.Quadrant.BR,
          NearestPoints.determineQuadrant(new Point(150, 150), reference));
      assertEquals(
          NearestPoints.Quadrant.TL,
          NearestPoints.determineQuadrant(new Point(50, 250), reference));
      assertEquals(
          NearestPoints.Quadrant.TR,
          NearestPoints.determineQuadrant(new Point(150, 250), reference));

      // on
      assertEquals(
          NearestPoints.Quadrant.MULTI,
          NearestPoints.determineQuadrant(new Point(100, 200), reference));

      // directly below
      assertEquals(
          NearestPoints.Quadrant.MULTI,
          NearestPoints.determineQuadrant(new Point(100, 100), reference));

      // directly above
      assertEquals(
          NearestPoints.Quadrant.MULTI,
          NearestPoints.determineQuadrant(new Point(100, 200), reference));

      // directly left
      assertEquals(
          NearestPoints.Quadrant.MULTI,
          NearestPoints.determineQuadrant(new Point(50, 200), reference));

      // directly right
      assertEquals(
          NearestPoints.Quadrant.MULTI,
          NearestPoints.determineQuadrant(new Point(150, 200), reference));
  }

  @Test
  public void testDetermineMultiQuandrant() {
      Point reference = new Point(10, 20);

      assertIterableEquals(
          NearestPoints.determineMultiQuadrants(new Point(1, 2), reference),
          Arrays.asList(NearestPoints.Quadrant.BL));

      assertIterableEquals(
          NearestPoints.determineMultiQuadrants(new Point(11, 21), reference),
          Arrays.asList(NearestPoints.Quadrant.TR));

      assertIterableEquals(
          NearestPoints.determineMultiQuadrants(new Point(10, 21), reference),
          Arrays.asList(NearestPoints.Quadrant.TL,
                        NearestPoints.Quadrant.TR));

      assertIterableEquals(
          NearestPoints.determineMultiQuadrants(new Point(10, 19), reference),
          Arrays.asList(NearestPoints.Quadrant.BL,
                        NearestPoints.Quadrant.BR));

      assertIterableEquals(
          NearestPoints.determineMultiQuadrants(new Point(11, 20), reference),
          Arrays.asList(NearestPoints.Quadrant.TR,
                        NearestPoints.Quadrant.BR));

      assertIterableEquals(
          NearestPoints.determineMultiQuadrants(new Point(9, 20), reference),
          Arrays.asList(NearestPoints.Quadrant.TL,
                        NearestPoints.Quadrant.BL));

      assertIterableEquals(
          NearestPoints.determineMultiQuadrants(new Point(10, 20), reference),
          Arrays.asList(NearestPoints.Quadrant.TL,
                        NearestPoints.Quadrant.BL,
                        NearestPoints.Quadrant.TR,
                        NearestPoints.Quadrant.BR));
  }

  @Test
  public void testOutOfBounds() {
      NearestPoints np = new NearestPoints();

      assertTrue(np.isOutOfBounds());

      // Keep the original out of bounds points.
      var original = np.points_;

      // Replace the point data with valid point values.
      // The actual values dont matter.r
      var p = new Point(0, 0);
      np.points_ = new Point[] {p, p, p, p};
      assertFalse(np.isOutOfBounds());

      // Replace each valid point with its out of bounds marker
      for (int i = 0; i < 4; i++) {
          np.points_[i] = original[i];
          assertTrue(np.isOutOfBounds());
          np.points_[i] = p;
      }
  }

  @Test
  public void testMaybeUpdatePoint() {
      // The initial point we're going to set
      Point tmp = new Point(100, 200);

      // The final points we're going to expect for each quadrant.
      final Point points[] = new Point[] {
        new Point(111, 221),
        new Point(112, 222),
        new Point(113, 223),
        new Point(114, 224),
      };

      // The final point distances we're going to expect.
      final double distances[] = new double [] {
        100.1, 200.2, 300.3, 400.4
      };

      // These are in numeric order so previous array's will align
      // when we check at the end.
      NearestPoints.Quadrant quadrants[] = new NearestPoints.Quadrant[] {
          NearestPoints.Quadrant.BL,
          NearestPoints.Quadrant.BR,
          NearestPoints.Quadrant.TL,
          NearestPoints.Quadrant.TR,
      };

      NearestPoints np = new NearestPoints();

      // Check each quadrant
      for (var q : quadrants) {
          int i = q.value();
          var d = distances[i];

          // Initial value will set.
          assertTrue(np.maybeUpdatePoint(tmp, d + 0.5, q));
          assertSame(tmp, np.points_[i]);
          assertEquals(d + 0.5, np.distances_[i]);

          // Farther distance wont change.
          var p = points[i];
          assertFalse(np.maybeUpdatePoint(p, d + 1, q));
          assertSame(tmp, np.points_[i]);
          assertEquals(d + 0.5, np.distances_[i]);

          // Same value wont change.
          assertFalse(np.maybeUpdatePoint(p, d + 0.5, q));
          assertSame(tmp, np.points_[i]);
          assertEquals(d + 0.5, np.distances_[i]);

          // Closer value will change.
          assertTrue(np.maybeUpdatePoint(p, d, q));
          assertSame(p, np.points_[i]);
          assertEquals(d, np.distances_[i]);
      }

      // Ensure maybeUpdatePoint did not also update wrong quadrants
      for (int i = 0; i < 4; i++) {
          assertEquals(distances[i], np.distances_[i]);
          assertSame(points[i], np.points_[i]);
      }
  }


  @Test
  public void testFindNearestPoints() {
      // 3 x 3 grid, not in order
      final var referencePoints = Arrays.asList(new Point[] {
          new Point(200, 100),
          new Point(200, 200),
          new Point(200, 300),

          new Point(300, 300),
          new Point(300, 200),
          new Point(300, 100),

          new Point(100, 100),
          new Point(100, 300),
          new Point(100, 200),
      });

      final var p = new Point(110, 220);
      var got = NearestPoints.findNearestPoints(p, referencePoints);
      assertFalse(got.isOutOfBounds());

      var points = got.points();
      var distances = got.distances();


      assertEquals(new Point(100, 300),
                   points[NearestPoints.Quadrant.TL.value()]);
      assertEquals(Math.sqrt(10 * 10 + 80 * 80),
                   distances[NearestPoints.Quadrant.TL.value()],
                   0.1);

      assertEquals(new Point(100, 200),
                   points[NearestPoints.Quadrant.BL.value()]);
      assertEquals(Math.sqrt(10 * 10 + 20 * 20),
                   distances[NearestPoints.Quadrant.BL.value()],
                   0.1);

      assertEquals(new Point(200, 300),
                   points[NearestPoints.Quadrant.TR.value()]);
      assertEquals(Math.sqrt(90 * 90 + 80 * 80),
                   distances[NearestPoints.Quadrant.TR.value()],
                   0.1);

      assertEquals(new Point(200, 200),
                   points[NearestPoints.Quadrant.BR.value()]);
      assertEquals(Math.sqrt(90 * 90 + 20 * 20),
                   distances[NearestPoints.Quadrant.BR.value()],
                   0.1);

      got = NearestPoints.findNearestPoints(new Point(50, 200),
                                            referencePoints);
      assertTrue(got.isOutOfBounds());
  }

  @Test
  public void testFindNearestPointsOnNonGrid() {
      // This is a diamond-ish with bounding box of
      //    offset points {a, b, c, d}
      // and testing points {p, q, r} as depicted here:
      //        ..a
      //      b     .
      //      .  pqr.
      //      .     d
      //        c..
      var a = new Point(160, 200);
      var b = new Point(100, 160);
      var c = new Point(140, 50);
      var d = new Point(200, 140);

      var referencePoints = Arrays.asList(new Point[] { a, b, c, d });

      // p is inside the reference points.
      var p = new Point(150, 150);
      var got = NearestPoints.findNearestPoints(p, referencePoints);
      var points = got.points();
      assertEquals(c, points[NearestPoints.Quadrant.BL.value()]);
      assertEquals(d, points[NearestPoints.Quadrant.BR.value()]);
      assertEquals(b, points[NearestPoints.Quadrant.TL.value()]);
      assertEquals(a, points[NearestPoints.Quadrant.TR.value()]);

      // q is on the same gridline as reference point a.
      var q = new Point(a.x(), p.y());

      got = NearestPoints.findNearestPoints(q, referencePoints);
      points = got.points();
      assertEquals(c, points[NearestPoints.Quadrant.BL.value()]);
      assertEquals(d, points[NearestPoints.Quadrant.BR.value()]);
      assertEquals(a, points[NearestPoints.Quadrant.TL.value()]);
      assertEquals(a, points[NearestPoints.Quadrant.TR.value()]);

      // r is just over to the right of a, so there is no longer a TR.
      var r = new Point(q.x() + 1, q.y());
      got = NearestPoints.findNearestPoints(r, referencePoints);
      assertTrue(got.isOutOfBounds());
  }

}
