package team1403.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import org.junit.jupiter.api.Test;

class TargetingCalculatorTest {
  class FakeCalculator extends TargetingCalculator {
    public FakeCalculator() {
      super(1000, 1000);
    }

    /**
     * Overrides the interpolation method since we do not have them.
     *
     * TODO: Remove these methods when the calculator implements them.
     */
    @Override
    protected double interpolateAngle(
      TargetingParams[] params, double[] distances) {
        angleParams = params;
        angleDistances = distances;
        return -1.1;
    }
    protected double interpolateSpeed(
      TargetingParams[] params, double[] distances) {
        speedParams = params;
        speedDistances = distances;
        return -2.2;
    }
  
    protected double interpolateHeading(
      TargetingParams[] params, double[] distances) {
        headingParams = params;
        headingDistances = distances;
        return -3.3;
    }

    public TargetingParams[] angleParams;
    public double[] angleDistances;

    public TargetingParams[] speedParams;
    public double[] speedDistances;

    public TargetingParams[] headingParams;
    public double[] headingDistances;
  }


  @Test
  public void testComputeParams() {
    var calc = new FakeCalculator();

    // Points are unordered.
    calc.addReferencePoint(new Point(200, 200),
                           new TargetingParams(2.2, 222.2, 22.2));
    calc.addReferencePoint(new Point(200, 300),
                           new TargetingParams(2.3, 222.3, 22.3));
    calc.addReferencePoint(new Point(200, 100),
                           new TargetingParams(2.1, 222.1, 22.1));


    calc.addReferencePoint(new Point(100, 300),
                           new TargetingParams(1.3, 111.3, 11.3));
    calc.addReferencePoint(new Point(100, 200),
                           new TargetingParams(1.2, 111.2, 11.2));
    calc.addReferencePoint(new Point(100, 100),
                           new TargetingParams(1.1, 111.1, 11.1));

    calc.addReferencePoint(new Point(300, 200),
                           new TargetingParams(3.2, 333.2, 33.2));
    calc.addReferencePoint(new Point(300, 100),
                           new TargetingParams(3.1, 333.1, 33.1));
    calc.addReferencePoint(new Point(300, 300),
                           new TargetingParams(3.3, 333.3, 33.3));


    var params = calc.computeParams(new Point(210, 180));
    assertEquals(-1.1, params.angle());
    assertEquals(-2.2, params.speed());
    assertEquals(-3.3, params.heading());

    assertSame(calc.angleParams, calc.speedParams);
    assertSame(calc.headingParams, calc.speedParams);

    assertSame(calc.angleDistances, calc.speedDistances);
    assertSame(calc.headingDistances, calc.speedDistances);


    var refs = calc.speedParams;  // reference points
    var distances = calc.speedDistances;  // from reference points
    // 200, 100
    assertEquals(222.1, refs[NearestPoints.Quadrant.BL.value()].speed());
    assertEquals(Math.sqrt(10*10 + 80*80),
                 distances[NearestPoints.Quadrant.BL.value()]);

    // 300, 100
    assertEquals(333.1, refs[NearestPoints.Quadrant.BR.value()].speed());
    assertEquals(Math.sqrt(90*90 + 80*80),
                 distances[NearestPoints.Quadrant.BR.value()]);

    // 200, 200
    assertEquals(222.2, refs[NearestPoints.Quadrant.TL.value()].speed());
    assertEquals(Math.sqrt(10*10 + 20*20),
                 distances[NearestPoints.Quadrant.TL.value()]);

    // 300, 200
    assertEquals(333.2, refs[NearestPoints.Quadrant.TR.value()].speed());
    assertEquals(Math.sqrt(90*90 + 20*20),
                 distances[NearestPoints.Quadrant.TR.value()]);
  }
}
