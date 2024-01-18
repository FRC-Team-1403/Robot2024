package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Hanger;

public class RunHanger extends Command {
    private Hanger m_hanger;
    private double m_hangerSpeed;

    public RunHanger(Hanger hanger, double hangerSpeed) {
        m_hanger = hanger;
        m_hangerSpeed = hangerSpeed;
    }

    @Override
    public void initialize() {
        m_hanger.setHangerSpeed(m_hangerSpeed);
    }

    @Override
    public void execute() {
        //hangs robot
    }

    @Override
    public boolean isFinished() { 
        if (m_hanger.isAtTop()) {
            m_hanger.setHangerSpeed(0);
            return true;
        }
        return false;
    }
}
