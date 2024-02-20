package team1403.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.Constants.Intake;
import team1403.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private IntakeSubsystem m_intake;
    private double m_speed;
    public IntakeCommand(IntakeSubsystem intake, double speed) {
        m_intake = intake;
        m_speed = speed;
    }
    @Override 
    public boolean isFinished() {
      boolean stop = !m_intake.isShooterGateOn();
      if (stop) {
        m_intake.setIntakeSpeed(0);
      }
      return stop;
    }
    @Override
    public void execute() {
      if (!m_intake.isIntakeGateOn()) {
        m_intake.setIntakeSpeed(m_speed / 10.0);
      }
      else m_intake.setIntakeSpeed(m_speed);
    }
}