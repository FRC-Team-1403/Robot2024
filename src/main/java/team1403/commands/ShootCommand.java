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
    private double rpm;
    private boolean ready = false;
    private boolean crossed = false;
    private int done = 60;

    public ShootCommand(IntakeSubsystem intake, double rpm) {
        m_intake = intake;
        this.rpm = rpm;
    }

    @Override
    public void initialize() {
        m_intake.setShooterRPM(rpm);
    }

    @Override
    public boolean isFinished() {
        // waits 4ms
        boolean isDone = done == 2;
        if (isDone) {
            m_intake.setIntakeSpeed(0);
            m_intake.setShooterSpeed(0);
        }
        return isDone;

    }

    @Override
    public void execute() {
        m_intake.setShooterRPM(rpm);
        if (ready || (m_intake.getShooterRPMBottom() > rpm / 1.1 &&  m_intake.getShooterRPMTop() > rpm / 1.1)) {
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
