package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
    private Shooter m_shooter;
    private double m_shooterSpeed;
    private double m_time;
    private double m_startTime;

    public RunShooter(Shooter shooter, double shooterSpeed, double time) {
        m_shooter = shooter;
        m_shooterSpeed = shooterSpeed;
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
        m_shooter.setShooterSpeed(m_shooterSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return Timer.getFPGATimestamp() - m_startTime >= m_time;
    }
}
