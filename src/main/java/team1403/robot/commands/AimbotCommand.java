package team1403.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.swerve.PhotonVisionCommand;

public class AimbotCommand extends Command {
    private PhotonVisionCommand m_PhotonVisionCommand;

    public AimbotCommand(PhotonVisionCommand photonVisionCommand){
        m_PhotonVisionCommand = photonVisionCommand;
    }

    @Override
    public void execute(){
        m_PhotonVisionCommand.turnRobotToTag();
        SmartDashboard.putBoolean("Cheats on?", m_PhotonVisionCommand.isFinished());
    }

    @Override
    public boolean isFinished(){
        return m_PhotonVisionCommand.isFinished();
    }
    
}
