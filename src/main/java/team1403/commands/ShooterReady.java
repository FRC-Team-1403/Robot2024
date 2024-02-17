package team1403.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.Constants.Intake;
import team1403.subsystems.IntakeSubsystem;

public class ShooterReady extends Command {
    private IntakeSubsystem m_intake;
    private double m_rpm;
    private boolean m_bReady;

    public ShooterReady(IntakeSubsystem intake, double rpm) {
        m_intake = intake;
        m_rpm = rpm;
    }

    @Override 
    public boolean isFinished() {
        if (m_intake.getShooterRpm() == m_rpm) {
            m_bReady = true;
        }
        else m_bReady = false;
        SmartDashboard.putBoolean("Shooter Ready", m_bReady);
        return m_bReady;
    }

    @Override
    public void execute() {
        m_intake.setShooterRpm(m_rpm);
    }
}
