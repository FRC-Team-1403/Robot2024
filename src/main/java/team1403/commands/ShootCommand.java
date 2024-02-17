package team1403.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.Constants.Intake;
import team1403.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {
    private IntakeSubsystem m_intake;
    private double m_intakeSpeed;
    private double m_shooterSpeed;

    public ShootCommand(IntakeSubsystem intake, double intakeSpeed, double shooterSpeed) {
        m_intake = intake;
        m_intakeSpeed = intakeSpeed;
        m_shooterSpeed = shooterSpeed;
    }
    
    @Override public boolean isFinished() {
       return m_intake.isShooterFinished();
    }

    @Override
    public void execute() {
        m_intake.setIntakeSpeed(m_intakeSpeed);
        m_intake.setShooterSpeed(m_shooterSpeed);
    }
}
