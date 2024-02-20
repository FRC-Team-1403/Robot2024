package team1403.robot.Datables;

public class ShooterValues {
    public double angle;
    public double rpm;
    public double robotAngle;
    public Object checkPlot;  
    public double thetaz;

    public ShooterValues(double angle, double rpm, double offset) {
        this.angle = angle;
        this.rpm = rpm;
        this.robotAngle = offset;
    }

    public String toString() {
        return "angle: " + angle + " rpm: " + rpm + " robotAngle: " + robotAngle;
    }
}