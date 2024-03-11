package team1403.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.HangerSubsystem;



public class RunHanger extends Command {
    private HangerSubsystem m_hanger;
    private boolean m_top;
    private Timer m_timer;

    public RunHanger(HangerSubsystem hanger) {
        m_hanger = hanger;
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_top = m_hanger.isAtTop();
        m_timer.reset();
        m_timer.start();
        if(!m_top)
        {
            m_hanger.unlockHanger();
        }
        else
        {
            m_hanger.lockHanger();
        }
    }
    
    @Override
    public void execute() {
        if(m_timer.hasElapsed(0.5))
        {
            if(!m_top)
                m_hanger.runHanger(0.5);
            else
                m_hanger.runHanger(-0.5);
        }
    }
    
    @Override
    public boolean isFinished() {
        if(m_top) return m_hanger.isAtBottom();
        return m_hanger.isAtTop();
    }
}
