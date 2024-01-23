package team1403.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PhotonVisionCommand extends Command {
    private Limelight m_Limelight;

    private final SwerveSubsystem m_swerve;     
    private final PIDController m_thetaController;
    private boolean isRotated = false;
    private final double m_rotationCutoff;

    private double m_vxspeed;
    private double m_vyspeed;

    public PhotonVisionCommand(Limelight limelight, SwerveSubsystem swerve, double rotationCutoff) {
        m_Limelight = limelight;
        m_swerve = swerve;
        m_rotationCutoff = rotationCutoff;
        m_thetaController = new PIDController(.1, 0, 0);
    }


    @Override
    public void initialize()
    {
        m_vxspeed = 0;
        m_vyspeed = 0;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("ZDistance", m_Limelight.getZDistance());
        SmartDashboard.putNumber("XDistance", m_Limelight.getXDistance());
        SmartDashboard.putNumber("DirectDistance", m_Limelight.getDistanceFromTarget());
        SmartDashboard.putNumber("ZAngle", m_Limelight.getZAngle());
        
    }

    @Override
    public boolean isFinished() {
        // If robot is within acceptable bounds for position & has been sucessfully
        // rotated
        return isRotated;
    }

    
    public void turnRobotToTag() {
        double rotationOfSwerve =m_swerve.getGyroscopeRotation().getDegrees();

        // If rotation is within acceptable bounds
        if (Math.abs(rotationOfSwerve) <= Math.abs(m_rotationCutoff)) {
          isRotated = true;
          return;
        } 


        double rotationalSpeed = m_thetaController.calculate(rotationOfSwerve, m_Limelight.getZAngle());
        SmartDashboard.putNumber("aimbot rotation",rotationalSpeed);
        // m_swerve.driveNoOffset(new ChassisSpeeds(m_vxspeed, m_vyspeed, rotationalSpeed));
    }
}
