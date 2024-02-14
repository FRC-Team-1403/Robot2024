package team1403.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.arm.Wrist;

public class RunWrist extends Command {
    private Wrist m_wrist;
    private double m_wristAngle;

    public RunWrist(Wrist wrist, double wristAngle) {
        m_wrist = wrist;
        m_wristAngle = m_wrist.limitAngle(wristAngle);
    }

    @Override
    public void initialize() {
        m_wrist.setWristAngle(m_wristAngle);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return m_wrist.isAtSetpoint();
    }
}
