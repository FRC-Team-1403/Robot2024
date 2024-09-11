package team1403.robot.swerve;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import team1403.lib.util.TimeDelayedBoolean;
import team1403.robot.Constants;

//adapted from team 254
public class SwerveHeadingCorrector {
    //initial rotation is unknown
    private Optional<Double> yaw_setpoint = Optional.empty();
    private PIDController m_controller = new PIDController(5, 0, 0);
    private TimeDelayedBoolean m_yawZeroDetector = new TimeDelayedBoolean();
    private LinearFilter m_gyroVelFilter = LinearFilter.singlePoleIIR(Constants.kLoopTime * 3, Constants.kLoopTime);


    public SwerveHeadingCorrector()
    {
        m_controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final static double OMEGA_THRESH = 0.03;

    public ChassisSpeeds update(ChassisSpeeds target, ChassisSpeeds cur_vel, Rotation2d gyro, double gyro_vel)
    {
        double current_rotation = MathUtil.angleModulus(gyro.getRadians());
        boolean is_translating = Math.hypot(target.vxMetersPerSecond, target.vyMetersPerSecond) > 0.1;
        //timeout can be lowered with a well tuned slew rate
        boolean is_near_zero = m_yawZeroDetector.update(Math.abs(target.omegaRadiansPerSecond) < OMEGA_THRESH, 0.2);
        boolean is_rotating = Math.abs(m_gyroVelFilter.calculate(gyro_vel)) > 1;
        /* gyro angular vel used when you get hit by another robot and rotate inadvertantly, don't want to snap heading back when that happens
          usually such a hit would create a high angular velocity temporarily, so check for that (units of degrees/s) */
        boolean auto_reset = Math.abs(target.omegaRadiansPerSecond) > OMEGA_THRESH || 
                                (!is_translating && is_near_zero) || 
                                is_rotating;

        Logger.recordOutput("Swerve Yaw Setpoint", yaw_setpoint.orElse(current_rotation));
        Logger.recordOutput("Swerve Yaw Setpoint Present", yaw_setpoint.isPresent());

        if(is_near_zero && is_rotating) {
            is_near_zero = false;
            m_yawZeroDetector.reset();
        }

        if(auto_reset && yaw_setpoint.isPresent()) {
            resetHeadingSetpoint();
        }

        if(is_near_zero && yaw_setpoint.isEmpty())
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

