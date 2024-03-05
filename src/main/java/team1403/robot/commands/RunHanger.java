package team1403.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.HangerSubsystem;



public class RunHanger extends Command {
    private HangerSubsystem m_hanger;

    public RunHanger(HangerSubsystem hanger) {
        m_hanger = hanger;
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
