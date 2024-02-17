package team1403.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.Constants.Intake;
import team1403.subsystems.IntakeSubsystem;

public class IntakeRollback extends Command {
    private IntakeSubsystem m_intake;
    public IntakeRollback(IntakeSubsystem intake) {
        m_intake = intake;
    }
    @Override 
    public boolean isFinished() {
      boolean stop = m_intake.isIntakeGateOn();
      if (stop) {
        m_intake.setIntakeSpeed(0);
      }
      return stop;
    }
    @Override
    public void execute() {
      m_intake.setIntakeSpeed(-0.1);
    }
}