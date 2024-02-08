package team1403.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Hanger;


public class RunHanger extends Command {
    private double m_hangerSpeed;
    private Hanger m_rightHangerMotor;
    private Hanger m_leftHangerMotor;

    public RunHanger(Hanger rightHangerMotor, Hanger leftHangerMotor, double hangerSpeed) {
        m_rightHangerMotor = rightHangerMotor;
        m_leftHangerMotor = leftHangerMotor;
        m_hangerSpeed = hangerSpeed;
    }

    @Override
    public void initialize() {
        m_rightHangerMotor.setHangerSpeed(m_hangerSpeed);
        m_leftHangerMotor.setHangerSpeed(m_hangerSpeed);
    }

    @Override
    public void execute() {
        //hangs robot
    }

    @Override
    public boolean isFinished() { 
        if (m_rightHangerMotor.isAtTop()) {
            m_rightHangerMotor.setHangerSpeed(0);
            m_leftHangerMotor.setHangerSpeed(0);
            return true;
        }
        return false;
    }
}
