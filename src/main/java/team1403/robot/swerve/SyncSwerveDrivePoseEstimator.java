package team1403.robot.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import team1403.robot.Constants;

public class SyncSwerveDrivePoseEstimator {

    private SwerveDrivePoseEstimator m_odometer;
    private Supplier<SwerveModulePosition[]> m_modPosSup;
    private Supplier<Rotation2d> m_gyroRotation;


    public SyncSwerveDrivePoseEstimator(Pose2d initialPose, Supplier<Rotation2d> gyroRotation, Supplier<SwerveModulePosition[]> modulePoses) {
        m_modPosSup = modulePoses;
        m_gyroRotation = gyroRotation;
        m_odometer = new SwerveDrivePoseEstimator(Constants.Swerve.kDriveKinematics, m_gyroRotation.get(), m_modPosSup.get(), initialPose);
    }

    public void update() {
        synchronized(this) {
            m_odometer.update(m_gyroRotation.get(), m_modPosSup.get());
        }
    }

    public void addVisionMeasurement(Pose2d measurement, double timestamp, Matrix<N3, N1> stdev) {
        synchronized(this) {
            m_odometer.addVisionMeasurement(measurement, timestamp, stdev);
        }
    }

    public void addVisionMeasurement(Pose2d measurement, double timestamp) {
        synchronized(this) {
            m_odometer.addVisionMeasurement(measurement, timestamp);
        }
    }

    public void resetPosition(Pose2d pose) {
        synchronized(this) {
            m_odometer.resetPosition(m_gyroRotation.get(), m_modPosSup.get(), pose);
        }
    }

    public Pose2d getPose() {
        synchronized(this) {
            return m_odometer.getEstimatedPosition();
        }
    }

    
}
