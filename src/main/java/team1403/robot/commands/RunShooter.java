package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RunShooter extends Command {
    private Shooter m_shooter;
    private Intake m_intake;

public class RunShooter extends Command {
    private Shooter m_shooter;
    private double m_shooterSpeed;
    private double m_intakeSpeed;
    private DigitalInput m_shooterPhotogate;

    public RunShooter(Shooter shooter, double shooterSpeed, Intake intake, double intakeSpeed, DigitalInput shooterPhotogate) {
        m_shooter = shooter;
        m_intake = intake;
        m_shooterSpeed = shooterSpeed;
        m_intakeSpeed = intakeSpeed;
        m_shooterPhotogate = shooterPhotogate;
    }

    @Override
    public void initialize() {
        m_shooter.setShooterSpeed(m_shooterSpeed);
    }

    @Override
    public void execute() {
        m_intake.setIntakeSpeed(m_intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        if (m_shooterPhotogate.get()) {
            new SequentialCommandGroup(
                new WaitCommand(3),
                new InstantCommand(() -> m_intake.stop(), m_intake),
                new InstantCommand(() -> m_shooter.stop(), m_shooter));
        }
        return m_shooterPhotogate.get();
    }
}
