package team1403.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

class TargetingParamsTest {

  @Test
  public void testConstructor() {
      var p = new TargetingParams(1.1, 2.2, 3.3);
      assertEquals(1.1, p.angle());
      assertEquals(2.2, p.speed());
      assertEquals(3.3, p.heading());
  }

  @Test
  public void testEqualsYes() {
      assertEquals(new TargetingParams(1.1, 2.2, 3.3),
                   new TargetingParams(1.1, 2.2, 3.3));
  }

  @Test
  public void testEqualsNo() {
      var p = new TargetingParams(1.1, 2.2, 3.3);

      // test individual components differing
      assertFalse(p == new TargetingParams(1, 2.2, 3.3));
      assertFalse(p == new TargetingParams(1.1, 2, 3.3));
      assertFalse(p == new TargetingParams(1.1, 2.2, 3));
  }

  @Test
  public void testNormalizeParams() {
    // (raw angle, normalized angle) test cases.
    double[][] testCases = {
        // non-negative angles
        {0, 0},
        {90, 90},
        {179, 179},
        {179.9, 179.9},
        {180, 180},
        {180.1, -179.9},
        {181, -179},
        {270, -90},
        {359, -1},
        {360, 0},
        {361, 1},
        {360 * 5 + 10, 10},
        {360 * 5 - 10, -10},

        // negative angles
        {-1, -1},
        {-90, -90},
        {-179, -179},
        {-179.9, -179.9},
        {-180, 180},
        {-180.1, 179.9},
        {-181, 179},
        {-360, 0},
        {-360 * 5 + 10, 10},
        {-360 * 5 - 10, -10},

        // small angles.
        {-0.1, -0.1},
        {0.1, 0.1},
    };


    for (var pair : testCases) {
        var p = new TargetingParams(pair[0], pair[0], pair[0]);
        assertEquals(pair[1], p.angle());
        assertEquals(pair[1], p.heading());
        assertEquals(pair[0], p.speed());
    }
  }
}
