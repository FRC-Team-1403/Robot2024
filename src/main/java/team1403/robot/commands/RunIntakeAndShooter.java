package team1403.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.IntakeAndShooter;

public class RunIntakeAndShooter extends Command {
    private IntakeAndShooter m_intakeAndShooter;
    private double m_shooterSpeed;
    private double m_time;
    private double m_startTime;

    public RunIntakeAndShooter(IntakeAndShooter intakeAndShooter, double shooterSpeed, double time) {
        m_intakeAndShooter = intakeAndShooter;
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
        m_intakeAndShooter.setShooterSpeed(m_shooterSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return Timer.getFPGATimestamp() - m_startTime >= m_time;
    }
}
