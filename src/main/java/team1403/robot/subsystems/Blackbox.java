package team1403.robot.subsystems;

public class Blackbox {
    //expand further later to include arm and wrist angles along with RPMs 
    private static boolean trigger;
    private static boolean auto_finished;
    private static boolean is_sim;

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

    public static boolean isSimulation()
    {
        return is_sim;
    }

    public static void setSimulation(boolean b)
    {
        is_sim = b;
    }
}
