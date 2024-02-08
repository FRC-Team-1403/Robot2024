package team1403.robot.Datables;

public class ShooterValues {
    public double angle;
    public double rpm;
    public double robotAngle;
    private final int regressionThreshold = 8;
    public ShooterValues interpolateOther(ShooterValues other, int selfDistance, int otherDistance) {
        if (selfDistance  == 0) {
            return this;
        }
        else if (otherDistance  == 0) {
            return other;
        }
        if (selfDistance > otherDistance && selfDistance <= regressionThreshold){
            this.angle = SimpleRegression.calc(this.angle, other.angle, otherDistance, selfDistance);
            this.robotAngle = SimpleRegression.calc(this.robotAngle, other.robotAngle, otherDistance, selfDistance);
            this.rpm = SimpleRegression.calc(this.rpm, other.rpm, otherDistance, selfDistance);
            return this;
        }else if(otherDistance > selfDistance && selfDistance <= regressionThreshold) {
            this.angle = SimpleRegression.calc(this.angle, other.angle, otherDistance, selfDistance);
            this.robotAngle = SimpleRegression.calc(this.robotAngle, other.robotAngle, otherDistance, selfDistance);
            this.rpm = SimpleRegression.calc(this.rpm, other.rpm, otherDistance, selfDistance);
            return this;
        }
        return new ShooterValues(interpolate(this.angle, selfDistance, other.angle, otherDistance),
        interpolate(this.rpm, selfDistance, other.rpm, otherDistance),
        interpolate(this.robotAngle, selfDistance, other.robotAngle, otherDistance));
     }

    public static double interpolate(double highData, int highDataDistance, double lowData, int lowDataDistance) {
        return ((highData / highDataDistance) + (lowData / lowDataDistance)) *
        (highDataDistance * lowDataDistance) / 4;
    }
    public ShooterValues(double angle, double rpm, double offset) {
        this.angle = angle;
        this.rpm = rpm;
        this.robotAngle = offset;
    }

    public String toString() {
        return "angle: " + angle + " rpm: " + rpm + " robotAngle: " + robotAngle;
    }
}