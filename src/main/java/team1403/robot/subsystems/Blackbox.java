package team1403.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team1403.robot.Constants.Setpoints;
import team1403.robot.commands.TriggerShotCommand;

public class Blackbox {
    //expand further later to include arm and wrist angles along with RPMs 
    private static boolean trigger = false;
    private static boolean loaded = false;
    public static Pose2d targetPosition = null;
    //requested setpoint must have intake speed of 0!
    public static SonicBlasterSetpoint requestedSetpoint = Setpoints.kDriveSetpoint;

    public static boolean getTrigger() {
        return trigger;
    }

    public static void setTrigger(boolean trig) {
        trigger = trig;
    }

    public static Command shoot() {
        return new TriggerShotCommand();
    }

    public static void setLoaded(boolean l) {
        loaded = l;
    }

    public static boolean isLoaded() {
        return true;
    }

    public static boolean isValidTargetPosition() {
        return targetPosition != null;
    }

    public static Command commandSetpoint(SonicBlasterSetpoint st) {
        return new InstantCommand(() -> requestedSetpoint = st);
    }
}
