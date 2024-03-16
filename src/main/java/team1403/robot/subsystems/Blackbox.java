package team1403.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

public class Blackbox {
    //expand further later to include arm and wrist angles along with RPMs 
    private static boolean trigger;

    @AutoLogOutput(key="Blackbox Trigger")
    public static boolean getTrigger() {
        return trigger;
    }

    public static void setTrigger(boolean trig) {
        trigger = trig;
    }
}
