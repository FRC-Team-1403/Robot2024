package team1403.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.Constants.Intake;
import team1403.subsystems.IntakeSubsystem;

public class RunIntakeCommand extends Command {
    private IntakeSubsystem m_intake;

    public RunIntakeCommand(IntakeSubsystem intake) {
        m_intake = intake;
       // SmartDashboard.putNumber("Intake Top Neo Motor", m_topNeoRPM);    
       // SmartDashboard.putNumber("Intake Bottom Neo Motor", m_bottomNeoRPM);
       // SmartDashboard.putNumber("Intake Talon Motor", m_talonRPM);
    }

    @Override
    public void execute() {
        Intake.isIntaked = SmartDashboard.getBoolean("isIntaked", false);
        Intake.isPrimed = SmartDashboard.getBoolean("isPrimed", false);
        
        if (!Intake.isIntaked) { //Check if gamepiece is in the intake
            m_intake.setIntakeRpm(5000);
            Intake.isIntaked = m_intake.isShooterGateOn();
        }
        if (!Intake.isPrimed && Intake.isIntaked) {
          m_intake.setIntakeSpeed(0.1);
          Intake.isPrimed = !m_intake.isShooterGateOn();
        } else {
          m_intake.setShooterRpm(6000);
          try {
            wait(3000);
          } catch (Exception e) {
            e.printStackTrace();
          }
          m_intake.setIntakeRpm(5000);
          try {
            wait(1000);
          } catch (Exception e) {
            e.printStackTrace();
          }
          m_intake.setIntakeRpm(0);
          m_intake.setShooterRpm(0);
          Intake.isPrimed = false;
          Intake.isIntaked = false;
        }
        
    }
}
