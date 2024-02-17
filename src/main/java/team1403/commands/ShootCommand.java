package team1403.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.Constants.Intake;
import team1403.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {
    private IntakeSubsystem m_intake;
    private boolean isIntaked;
    public ShootCommand(IntakeSubsystem intake) {
        m_intake = intake;
    }
    @Override public boolean isFinished() {
       return m_intake.isIntakeGateOn() && m_intake.isShooterGateOn();
    }

    @Override
    public void execute() {
        m_intake.setIntakeSpeed(.001);
            m_intake.setShooterRpm(1000);;
        // if (m_intake.isIntakeGateOn() && m_intake.isShooterGateOn()) {
        //     m_intake.setShooterRpm(0);
        //     }
    }
}
