package team1403.robot.subsystems;

public class Blackbox {
    //expand further later to include arm and wrist angles along with RPMs 
    private static boolean trigger;

    public static boolean getTrigger() {
        return trigger;
    }

    public static void setTrigger(boolean trig) {
        trigger = trig;
    }
}
