package team1403.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PhotonVisionCommand extends Command {
    private Limelight m_Limelight;


    public PhotonVisionCommand(Limelight limelight) {
        m_Limelight = limelight;
    }


    @Override
    public void execute() {
        SmartDashboard.putNumber("ZDistance", m_Limelight.getZDistance());
        SmartDashboard.putNumber("XDistance", m_Limelight.getXDistance());
        SmartDashboard.putNumber("DirectDistance", m_Limelight.getYDistance());
        SmartDashboard.putNumber("ZAngle", m_Limelight.getZAngle());
        // turnRobotToTag();
    }

}
