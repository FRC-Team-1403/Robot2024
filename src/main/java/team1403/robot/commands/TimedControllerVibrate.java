package team1403.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class TimedControllerVibrate extends Command {

    private Timer m_t;
    private XboxController m_controller;
    private double m_seconds;
    private double m_strength;
    

    //strength is from 0-1
    public TimedControllerVibrate(XboxController c, double seconds, double strength) {
        m_controller = c;
        m_seconds = seconds;
        m_strength = strength;
        m_t = new Timer();
    }

    @Override
    public void initialize() {
        m_t.restart();
        m_controller.setRumble(RumbleType.kBothRumble, m_strength);
    }

    @Override
    public void end(boolean interrupt) {
        m_controller.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return m_t.hasElapsed(m_seconds);
    }    
}
