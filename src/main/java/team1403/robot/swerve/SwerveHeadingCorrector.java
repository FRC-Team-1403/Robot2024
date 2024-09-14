package team1403.robot.swerve;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import team1403.lib.util.TimeDelayedBoolean;
import team1403.robot.Constants;

//adapted from team 254
public class SwerveHeadingCorrector {
    //initial rotation is unknown
    private Optional<Double> yaw_setpoint = Optional.empty();
    private ProfiledPIDController m_controller = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(Constants.Swerve.kMaxAngularSpeed, 80));
    private TimeDelayedBoolean m_yawZeroDetector = new TimeDelayedBoolean();
    private LinearFilter m_gyroVelFilter = LinearFilter.singlePoleIIR(Constants.kLoopTime * 3, Constants.kLoopTime);
    private ChassisSpeeds m_retSpeeds = new ChassisSpeeds();


    public SwerveHeadingCorrector()
    {
        m_controller.reset(0);
        m_controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final static double OMEGA_THRESH = 0.03;

    public ChassisSpeeds update(ChassisSpeeds target, ChassisSpeeds cur_vel, Rotation2d gyro, double gyro_vel)
    {
        double current_rotation = MathUtil.angleModulus(gyro.getRadians());
        boolean is_translating = Math.hypot(target.vxMetersPerSecond, target.vyMetersPerSecond) > 0.1;
        //timeout can be lowered with a well tuned slew rate
        boolean is_near_zero = m_yawZeroDetector.update(Math.abs(target.omegaRadiansPerSecond) < OMEGA_THRESH, 0.2);
        double filtered_ang_vel = m_gyroVelFilter.calculate(gyro_vel);
        boolean is_rotating = Math.abs(filtered_ang_vel) > 2;
        /* gyro angular vel used when you get hit by another robot and rotate inadvertantly, don't want to snap heading back when that happens
          usually such a hit would create a high angular velocity temporarily, so check for that (units of degrees/s) */
        boolean auto_reset = Math.abs(target.omegaRadiansPerSecond) > OMEGA_THRESH || 
                                is_rotating;

        Logger.recordOutput("Swerve Yaw Setpoint", yaw_setpoint.orElse(current_rotation));
        Logger.recordOutput("Swerve Yaw Setpoint Present", yaw_setpoint.isPresent());
        Logger.recordOutput("Swerve Ang Vel Filtered", filtered_ang_vel);

        if(auto_reset && yaw_setpoint.isPresent()) {
            resetHeadingSetpoint();
        }

        if(is_near_zero && yaw_setpoint.isEmpty())
        {
            yaw_setpoint = Optional.of(current_rotation);
            m_controller.reset(current_rotation, cur_vel.omegaRadiansPerSecond);
        }
        else if(is_translating && yaw_setpoint.isPresent())
        {
            m_retSpeeds.vxMetersPerSecond = target.vxMetersPerSecond;
            m_retSpeeds.vyMetersPerSecond = target.vyMetersPerSecond;
            m_retSpeeds.omegaRadiansPerSecond = m_controller.calculate(current_rotation, yaw_setpoint.get());
            return m_retSpeeds;
        }

        return target;
    }

    public void resetHeadingSetpoint() {
        yaw_setpoint = Optional.empty();
        m_yawZeroDetector.reset();
    }
}

