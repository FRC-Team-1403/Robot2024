package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.swerve.PhotonVisionCommand;

public class AimbotCommand extends Command {
    private PhotonVisionCommand m_limelightCommand;

    public AimbotCommand(PhotonVisionCommand limelight) {
        m_limelightCommand = limelight;
    }

    @Override
    public void execute() {
        m_limelightCommand.turnRobotToTag();
    }

    @Override
    public boolean isFinished() {
        return m_limelightCommand.getRotationSpeedOfSwerve() == 0.0;
    }

}
