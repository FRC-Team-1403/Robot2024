package team1403.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import team1403.robot.Constants.Setpoints;

public class Blackbox {
    //expand further later to include arm and wrist angles along with RPMs 
    private static boolean trigger;
    private static boolean auto_finished;
    public static Pose2d targetPosition = null;
    //requested setpoint must have intake speed of 0!
    public static SonicBlasterSetpoint requestedSetpoint = Setpoints.kDriveSetpoint;

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
        return targetPosition != null;
    }
}
