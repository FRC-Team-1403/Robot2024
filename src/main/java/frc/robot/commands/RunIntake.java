package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    private Intake m_intake;
    private double m_intakeSpeed;
    private double m_time;
    private double m_startTime;

    public RunIntake(Intake intake, double intakeSpeed, double time) {
        m_intake = intake;
        m_intakeSpeed = intakeSpeed;
        m_time = time;
    }

    @Override
    public void initialize()
    {
        m_startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute()
    {
        m_intake.setIntakeSpeed(m_intakeSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return Timer.getFPGATimestamp() - m_startTime >= m_time;
    }
}
