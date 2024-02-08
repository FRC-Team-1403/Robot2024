package team1403.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.IntakeAndShooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RunShooter extends Command {
    private IntakeAndShooter m_shooter;
    private double m_shooterSpeed;
    private DigitalInput m_shooterPhotogate;

    public RunShooter(IntakeAndShooter shooter, double shooterSpeed, DigitalInput shooterPhotogate) {
        m_shooter = shooter;
        m_shooterSpeed = shooterSpeed;
        m_shooterPhotogate = shooterPhotogate;
    }

    @Override
    public void initialize() {
        m_shooter.setShooterSpeed(m_shooterSpeed);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        if (m_shooterPhotogate.get()) {
            new SequentialCommandGroup(
                new WaitCommand(3),
                new InstantCommand(() -> m_shooter.shooterStop(), m_shooter),
                new InstantCommand(() -> m_shooter.intakeStop(), m_shooter));
        }
        return m_shooterPhotogate.get();
    }
}