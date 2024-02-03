package team1403.robot.commands;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Wrist;

public class RunWrist extends Command {
    private Wrist m_wrist;
    private double m_wristSpeed;
    private double m_wristAngle;
    private DutyCycleEncoder m_wristBoreEncoder;

    public RunWrist(Wrist wrist, double wristSpeed, double wristAngle) {
        m_wrist = wrist;
        m_wristSpeed = wristSpeed;
        m_wristAngle = wristAngle;
    }

    @Override
    public void initialize() {
        m_wrist.setWristSpeed(m_wristSpeed);
        m_wristBoreEncoder.reset();
    }

    @Override
    public void execute() {
        m_wrist.setWristSpeed(m_wristSpeed);       //update wrist speed later
        //if(m_wristBoreEncoder.get() = Constants.Wrist.topLimit){

    }
    

    @Override
    public boolean isFinished() {
        if (m_wristBoreEncoder.get() > 90 || m_wristBoreEncoder.get() < 0);
            return true;
    }
}
