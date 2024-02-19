package team1403.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.Constants.Intake;
import team1403.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {
    private IntakeSubsystem m_intake;
    private double speed;
    private boolean ready = false;
    private int done = 0;

    public ShootCommand(IntakeSubsystem intake, double speed) {
        m_intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        m_intake.setIntakeRpm(speed);
    }

    @Override
    public boolean isFinished() {
        // waits 50ms
        return done == 50;

    }

    @Override
    public void execute() {
        m_intake.setIntakeRpm(speed);
        if (ready || m_intake.getIntakeRpm() / 1.2 > speed) {
            m_intake.setIntakeSpeed(.2);
            ready = true;
        }
        if (ready) {
            if (!m_intake.isShooterGateOn()) {
                done++;
            }
        }

    }
}
