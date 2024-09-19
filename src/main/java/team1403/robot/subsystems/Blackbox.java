package team1403.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class Blackbox {
    //expand further later to include arm and wrist angles along with RPMs 
    private static boolean trigger;
    private static boolean auto_finished;
    public static Pose2d targetPosition = new Pose2d();
    private static final Pose2d kZero = new Pose2d();

    public static boolean getTrigger() {
        return trigger;
    }

    public static boolean getAutoFinished() {
        return auto_finished;
    }

    public static void setTrigger(boolean trig) {
        trigger = trig;
    }

    public static void setAutoFinished(boolean finished) {
        auto_finished = finished;
    }

    public static boolean isValidTargetPosition() {
        return !targetPosition.equals(kZero);
    }
}
