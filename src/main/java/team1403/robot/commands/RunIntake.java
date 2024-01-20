package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Intake;

public class RunIntake extends Command {
    private Intake m_intake;
    private double m_intakeSpeed;

    public RunIntake(Intake intake, double intakeSpeed) {
        m_intake = intake;
        m_intakeSpeed = intakeSpeed;
    }

    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(m_intakeSpeed);
    }

    @Override
    public void execute() {
        //takes in note
    }

    @Override
    public boolean isFinished() {
        return m_intake.intakeReady();
    }
}
