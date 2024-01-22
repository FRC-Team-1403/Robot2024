package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Wrist;

public class RunWrist extends Command {
    private Wrist m_wrist;
    private double m_wristSpeed;

    public RunWrist(Wrist wrist, double wristSpeed) {
        m_wrist = wrist;
        m_wristSpeed = wristSpeed;
    }

    @Override
    public void initialize() {
        m_wrist.setWristSpeed(m_wristSpeed);
    }

    @Override
    public void execute() {
        //moves wrist
    }

    @Override
    public boolean isFinished() {
        return true; //leave as "true" so there's no error
        //intake or shoot
    }
}
