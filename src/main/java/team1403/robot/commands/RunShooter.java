package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Shooter;

public class RunShooter extends Command {
    private Shooter m_shooter;
    private double m_shooterSpeed;

    public RunShooter(Shooter shooter, double shooterSpeed) {
        m_shooter = shooter;
        m_shooterSpeed = shooterSpeed;
    }

    @Override
    public void initialize() {
        m_shooter.setShooterSpeed(m_shooterSpeed);
    }

    @Override
    public void execute() {
        //shoots note
    }

    @Override
    public boolean isFinished() {
        return m_shooter.shooterReady();
    }
}
