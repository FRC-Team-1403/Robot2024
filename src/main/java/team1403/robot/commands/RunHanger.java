package team1403.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Hanger;


public class RunHanger extends Command {
    private double m_hangerSpeed;
    private Hanger m_definiteHangerMotor;
    private Hanger m_possibleHangerMotor;

    public RunHanger(Hanger definiteHangerMotor, Hanger possibleHangerMotor, double hangerSpeed) {
        m_definiteHangerMotor = definiteHangerMotor;
        m_possibleHangerMotor = possibleHangerMotor;
        m_hangerSpeed = hangerSpeed;
    }

    @Override
    public void initialize() {
        m_definiteHangerMotor.setHangerSpeed(m_hangerSpeed);
        m_possibleHangerMotor.setHangerSpeed(m_hangerSpeed);
    }

    @Override
    public void execute() {
        //hangs robot
    }

    @Override
    public boolean isFinished() { 
        if (m_definiteHangerMotor.isAtTop()) {
            m_definiteHangerMotor.setHangerSpeed(0);
            return true;
        }
        return false;
    }
}
