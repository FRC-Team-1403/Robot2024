package team1403.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private IntakeSubsystem m_intake;
    private double m_topNeoRPM = 0;
    private double m_bottomNeoRPM = 0;
    private double m_talonRPM = 0;

    public IntakeCommand(IntakeSubsystem intake) {
        m_intake = intake;
       // SmartDashboard.putNumber("Intake Top Neo Motor", m_topNeoRPM);    
       // SmartDashboard.putNumber("Intake Bottom Neo Motor", m_bottomNeoRPM);
       // SmartDashboard.putNumber("Intake Talon Motor", m_talonRPM);
    }

    @Override
    public void execute() {
       // m_topNeoRPM = SmartDashboard.getNumber("Intake Top Neo Motor", m_topNeoRPM);
       // m_bottomNeoRPM = SmartDashboard.getNumber("Intake Bottom Neo Motor", m_bottomNeoRPM);
        // m_talonRPM = SmartDashboard.getNumber("Intake Talon Motor", m_talonRPM);
        // m_intake.setTopNeoSpeed(m_topNeoRPM);
        // m_intake.setBottomNeoSpeed(m_bottomNeoRPM);
        // m_intake.setTalonSpeed(m_talonRPM);
        
    }
}
