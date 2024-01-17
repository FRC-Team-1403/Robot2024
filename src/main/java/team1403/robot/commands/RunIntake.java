package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.IntakeAndShooter;

public class RunIntake extends Command {
    private IntakeAndShooter m_intakeAndShooter;
    private double m_intakeAndShooterSpeed;

    public RunIntake(IntakeAndShooter intakeAndShooter, double intakeSpeed) {
        m_intakeAndShooter = intakeAndShooter;
        m_intakeAndShooterSpeed = intakeSpeed;
    }

    @Override
    public void initialize() {
        m_intakeAndShooter.setShooterSpeed(m_intakeAndShooterSpeed);
    }

    @Override
    public void execute() {
        //intake note
    }

    @Override
    public boolean isFinished() {
        return m_intakeAndShooter.intakeReady();
    }
}