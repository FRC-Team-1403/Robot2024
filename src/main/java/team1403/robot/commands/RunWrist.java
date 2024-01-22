package team1403.robot.commands;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Wrist;

public class RunWrist extends Command {
    private Wrist m_wrist;
    private double m_wristSpeed;
    private DutyCycleEncoder m_wristAbsoluteEncoder;

    public RunWrist(Wrist wrist, double wristSpeed, double wristAngle) {
        m_wrist = wrist;
        m_wristSpeed = wristSpeed;
    }

    @Override
    public void initialize() {
        m_wrist.setWristSpeed(m_wristSpeed);
        m_wristAbsoluteEncoder.reset();
    }

    @Override
    public void execute() {
        //moves wrist
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_wristAbsoluteEncoder.getAbsolutePosition()) >= 90);
            return true;
    }
}
