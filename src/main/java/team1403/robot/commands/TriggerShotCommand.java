package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.arm.Wrist;
import team1403.robot.subsystems.Blackbox;

public class TriggerShotCommand extends Command {
    private IntakeAndShooter m_eff;
    private Wrist m_wrist;

    public TriggerShotCommand(IntakeAndShooter eff, Wrist wrist)
    {
        m_eff = eff;
        //sometimes wrist is blocking
        m_wrist = wrist;
    }

    public void initialize()
    {
        Blackbox.setTrigger(true);
    }
    
    public boolean isFinished()
    {
        return m_eff.isReady() && m_wrist.isAtSetpoint() && !m_eff.isShooterPhotogateTriggered();
    }
}
