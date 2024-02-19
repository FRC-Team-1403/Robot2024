package team1403.commands;

import javax.swing.tree.TreeCellRenderer;

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
    private boolean crossed = false;
    private int done = 0;

    public ShootCommand(IntakeSubsystem intake, double speed) {
        m_intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        m_intake.setShooterRpm(speed);
    }

    @Override
    public boolean isFinished() {
        // waits 4ms
        boolean isDone = done == 2;
        if (isDone) {
            m_intake.setEverythingSpeed(0);
        }
        return isDone;

    }

    @Override
    public void execute() {
        m_intake.setShooterRpm(speed);
        if (ready || (m_intake.getShooterRpmBottom() > speed / 1.2 &&  m_intake.getShooterRpmTop() > speed / 1.2)) {
            m_intake.setIntakeSpeed(1);
            ready = true;
        }
        if (ready) {
            if (!m_intake.isShooterGateOn() || !m_intake.isIntakeGateOn()) {
                crossed = true;
            }
            if (crossed)
                done++;
        }

    }
}
