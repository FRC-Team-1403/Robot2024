package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.IntakeAndShooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RunShooter extends Command {
    private IntakeAndShooter m_shooter;
    private double m_shooterSpeed;

    public RunShooter(IntakeAndShooter shooter, double shooterSpeed) {
        m_shooter = shooter;
        m_shooterSpeed = shooterSpeed;
    }

    @Override
    public void initialize() {
        new WaitCommand(1); // TODO: test time
        m_shooter.setShooterSpeed(m_shooterSpeed);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        if (!m_shooter.isShooterPhotogateTriggered()) {
            new SequentialCommandGroup(
                new WaitCommand(1), // TODO: test time
                new InstantCommand(() -> m_shooter.shooterStop(), m_shooter)
            );
        }
        return !m_shooter.isShooterPhotogateTriggered();
    }
}