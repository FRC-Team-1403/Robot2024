package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.IntakeAndShooter;

public class RunIntake extends Command {
    private IntakeAndShooter m_intake;
    private double m_intakeSpeed;

    public RunIntake(IntakeAndShooter intake, double intakeSpeed) {
        m_intake = intake;
        m_intakeSpeed = intakeSpeed;
    }

    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(m_intakeSpeed);
    }

    @Override
    public void execute() {
        //intakes note
    }

    @Override
    public boolean isFinished() {
        return m_intake.intakeReady();
    }
}