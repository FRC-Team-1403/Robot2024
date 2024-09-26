package team1403.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class SimSwerveModule extends SubsystemBase implements ISwerveModule {

    private final String m_name;
    private final SwerveModuleState m_state;
    private final SwerveModulePosition m_position;
    private double m_steerAngle;
    private double m_driveVel;
    private double m_drivePos;


    public SimSwerveModule(String name) {
        m_name = name;
        m_position = new SwerveModulePosition();
        m_state = new SwerveModuleState();
    }

    @Override
    public String getName() {
        return m_name;
    }

    @Override
    public SwerveModuleState getState() {
        m_state.angle = Rotation2d.fromRadians(m_steerAngle);
        m_state.speedMetersPerSecond = m_driveVel;

        return m_state;
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        m_position.angle = Rotation2d.fromRadians(m_steerAngle);
        m_position.distanceMeters = m_drivePos;

        return m_position;
    }

    @Override
    public void set(double driveMetersPerSecond, double steerAngle) {
        m_driveVel = driveMetersPerSecond;
        m_steerAngle = steerAngle;
    }

    @Override
    public void simulationPeriodic() {
        m_drivePos += m_driveVel * Constants.kLoopTime;
    }
    
}
