package team1403.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import team1403.robot.Constants.Setpoints;
import team1403.robot.subsystems.arm.RobotSetpoint;

public class Blackbox {
    private static boolean trigger;
    private static boolean autoTrigger;
    private static boolean autoFinished;
    private static double shooterRPM;
    private static double armAngle;
    private static double wristAngle;
    private static boolean isLoaded;
    private static RobotSetpoint targetSetpoint = Setpoints.kDefaultShoot;

    public static boolean getTrigger() {
        return trigger;
    }

    public static boolean getAutoFinished() {
        return autoFinished;
    }

    //triggers the intake state machine to shoot
    public static void setTrigger(boolean trig) {
        trigger = trig;
    }

    //triggers the default state machine to shoot in auto
    public static void setAutoTrigger(boolean trig) {
        autoTrigger = trig;
    }

    public static boolean getAutoTrigger() {
        return autoTrigger;
    }

    public static void setAutoFinished(boolean finished) {
        autoFinished = finished;
    }

    public static void setShooterRPM(double RPM) {
        shooterRPM = MathUtil.clamp(RPM, 0, 6000);
    }

    public static double getShooterRPM() {
        return shooterRPM;
    }

    public static void setArmAngle(double angle) {
        armAngle = angle;
    }

    public static void setWristAngle(double angle) {
        wristAngle = angle;
    }

    public static double getArmAngle() {
        return armAngle;
    }

    public static double getWristAngle() {
        return wristAngle;
    }

    public static boolean getIsLoaded() {
        return isLoaded;
    }

    public static void setIsLoaded(boolean loaded) {
        isLoaded = loaded;
    }

    public static void setTargetSetpoint(RobotSetpoint setpoint) {
        targetSetpoint = setpoint;
    }

    public static RobotSetpoint getTargetSetpoint() {
        return targetSetpoint;
    }
}
