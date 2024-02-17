package team1403.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.Constants.Intake;
import team1403.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private IntakeSubsystem m_intake;
    public IntakeCommand(IntakeSubsystem intake) {
        m_intake = intake;
    }
    @Override public boolean isFinished() {
      boolean stop = !m_intake.isShooterGateOn();
      if (stop) {
        m_intake.setIntakeSpeed(0);
      }
      return stop;
    }

    @Override
    public void execute() {
            m_intake.setIntakeSpeed(1);
            if (!m_intake.isIntakeGateOn()) {
              m_intake.setIntakeSpeed(0.2);
            }
    }
}
