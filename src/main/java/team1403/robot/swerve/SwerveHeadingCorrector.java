package team1403.robot.swerve;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import team1403.lib.util.TimeDelayedBoolean;

//adapted from team 254
public class SwerveHeadingCorrector {
    //initial rotation is unknown
    private Optional<Double> yaw_setpoint = Optional.empty();
    private PIDController m_controller = new PIDController(5, 0, 0);
    private TimeDelayedBoolean m_yawZeroDetector = new TimeDelayedBoolean();


    public SwerveHeadingCorrector()
    {
        m_controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final double OMEGA_THRESH = 0.03;

    public ChassisSpeeds update(double timestamp, ChassisSpeeds target, ChassisSpeeds cur_vel, Rotation2d gyro)
    {
        double current_rotation = MathUtil.angleModulus(gyro.getRadians());
        boolean is_translating = Math.hypot(target.vxMetersPerSecond, target.vyMetersPerSecond) > 0.1;
        boolean is_near_zero = m_yawZeroDetector.update(Math.abs(cur_vel.omegaRadiansPerSecond) < OMEGA_THRESH, 0.15);

        Logger.recordOutput("Swerve Yaw Setpoint", yaw_setpoint.orElse(current_rotation));

        if(!is_near_zero || Math.abs(target.omegaRadiansPerSecond) > OMEGA_THRESH || yaw_setpoint.isEmpty())
        {
            yaw_setpoint = Optional.of(current_rotation);
        }
        else if(is_translating && yaw_setpoint.isPresent())
        {
            double new_yaw = m_controller.calculate(current_rotation, yaw_setpoint.get());

            return new ChassisSpeeds(target.vxMetersPerSecond, target.vyMetersPerSecond, new_yaw);
        }

        return target;
    }

    public void resetHeadingSetpoint() {
        yaw_setpoint = Optional.empty();
    }
}

