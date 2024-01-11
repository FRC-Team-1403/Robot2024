package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hanger;

public class RunHanger extends Command {
    private Hanger m_hanger;
    private double m_hangerSpeed;
    private double m_time;
    private double m_startTime;

    public RunHanger(Hanger hanger, double hangerSpeed, double time) {
        m_hanger = hanger;
        m_hangerSpeed = hangerSpeed;
        m_time = time;
    }

    @Override
    public void initialize()
    {
        m_startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute()
    {
        m_hanger.setHangerSpeed(m_hangerSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return Timer.getFPGATimestamp() - m_startTime >= m_time;
    }
}
