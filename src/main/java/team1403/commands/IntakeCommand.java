package team1403.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private IntakeSubsystem m_intake;
    private double m_speed;
    public IntakeCommand(IntakeSubsystem intake, double speed) {
        m_intake = intake;
        m_speed = speed;
    }
    @Override public boolean isFinished() {
      boolean stop = m_intake.isShooterSwitchTripped();
      if (stop) {
        m_intake.setIntakeSpeed(0);
      }
      return stop;
    }

    @Override
    public void execute() {
      m_intake.setIntakeSpeed(m_speed);
      if (m_intake.isIntakeSwitchTripped()) {
        m_intake.setIntakeSpeed(m_speed / 3.0);
      }
    }
}
