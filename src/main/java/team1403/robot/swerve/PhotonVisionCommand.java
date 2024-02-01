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

    @Override
    public boolean isFinished() {
        // If robot is within acceptable bounds for position & has been sucessfully
        // rotated
        return isRotated;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(new ChassisSpeeds(), new Translation2d());
    }
    
    public void turnRobotToTag() {
 
        double rotationalSpeed = getRotationSpeedOfSwerve();
        m_swerve.drive(
          new ChassisSpeeds(0, 0, rotationalSpeed), 
          new Translation2d());
    }

    public double getRotationSpeedOfSwerve() {

        double rotationOfSwerve = m_swerve.getGyroscopeRotation().getDegrees();
        
        // If rotation is within acceptable bounds
        if (Math.abs(rotationOfSwerve - 180) <= 0.2) {
          isRotated = true;
          return 0.0;
        } 
    
        return m_thetaController.calculate(rotationOfSwerve, m_Limelight.getZAngle());
    }

}
