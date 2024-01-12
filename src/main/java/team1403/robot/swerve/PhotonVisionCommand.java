package team1403.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PhotonVisionCommand extends Command {
    private Limelight m_Limelight;

    private final SwerveSubsystem m_swerve;

    private final PIDController m_yawController;
    private final PIDController m_distanceController;
    private final PIDController m_thetaController;
    private boolean isRotated = false;

    public PhotonVisionCommand(Limelight limelight, SwerveSubsystem swerve) {
        m_Limelight = limelight;
        m_swerve = swerve;

        m_yawController = new PIDController(12, 0, 0.5);
        m_distanceController = new PIDController(12, 0, 0.5);
        m_thetaController = new PIDController(4, 0, 0);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("ZDistance", m_Limelight.getZDistance());
        SmartDashboard.putNumber("XDistance", m_Limelight.getXDistance());
        SmartDashboard.putNumber("DirectDistance", m_Limelight.getDistanceFromTarget());
        SmartDashboard.putNumber("ZAngle", m_Limelight.getZAngle());
        
        turnRobotToTag();
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
        double rotationOfSwerve = m_swerve.getGyroscopeRotation().getDegrees();

        // If rotation is within acceptable bounds
        if (Math.abs(rotationOfSwerve - 180) <= 0.2) {
          isRotated = true;
          return;
        } 
    
        double rotationalSpeed = m_thetaController.calculate(rotationOfSwerve, m_Limelight.getZAngle());
        m_swerve.drive(
          new ChassisSpeeds(0, 0, rotationalSpeed), 
          new Translation2d());
        

    }
}
