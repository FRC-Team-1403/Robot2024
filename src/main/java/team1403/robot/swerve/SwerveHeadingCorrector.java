package team1403.robot.swerve;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import monologue.Logged;
import team1403.lib.util.CougarLogged;
import team1403.lib.util.TimeDelayedBoolean;
import team1403.robot.Constants;

//adapted from team 254
public class SwerveHeadingCorrector implements CougarLogged {
    //initial rotation is unknown
    private Optional<Double> yaw_setpoint = Optional.empty();
    private PIDController m_controller = new PIDController(5, 0, 0);
    private TimeDelayedBoolean m_yawZeroDetector = new TimeDelayedBoolean();
    private LinearFilter m_gyroVelFilter = LinearFilter.singlePoleIIR(Constants.kLoopTime * 5, Constants.kLoopTime);
    private ChassisSpeeds m_retSpeeds = new ChassisSpeeds();


    public SwerveHeadingCorrector()
    {
        m_controller.enableContinuousInput(-Math.PI, Math.PI);

        if (Constants.DEBUG_MODE) {
            Constants.kDebugTab.add("SwerveHC PID", m_controller);
        }
    }

    private final static double OMEGA_THRESH = 0.03;

    public ChassisSpeeds update(ChassisSpeeds target, ChassisSpeeds cur_vel, Rotation2d gyro, double gyro_vel)
    {
        double current_rotation = MathUtil.angleModulus(gyro.getRadians());
        boolean is_translating = Math.hypot(target.vxMetersPerSecond, target.vyMetersPerSecond) > 0.1;
        //timeout can be lowered with a well tuned slew rate
        boolean is_near_zero = m_yawZeroDetector.update(Math.abs(target.omegaRadiansPerSecond) < OMEGA_THRESH, 0.2);
        double filtered_ang_vel = m_gyroVelFilter.calculate(gyro_vel);
        // degrees per second
        boolean is_rotating = Math.abs(filtered_ang_vel) > 8;
        /* gyro angular vel used when you get hit by another robot and rotate inadvertantly, don't want to snap heading back when that happens
          usually such a hit would create a high angular velocity temporarily, so check for that (units of degrees/s) */
        boolean auto_reset = Math.abs(target.omegaRadiansPerSecond) > OMEGA_THRESH ||
                                is_rotating;


        if(auto_reset && yaw_setpoint.isPresent()) {
            resetHeadingSetpoint();
        }

        log("SwerveHC/Yaw Setpoint", yaw_setpoint.orElse(current_rotation));
        log("SwerveHC/Yaw Setpoint Present", yaw_setpoint.isPresent());
        log("SwerveHC/Ang Vel Filtered", filtered_ang_vel);

        if(is_near_zero && yaw_setpoint.isEmpty())
        {
            yaw_setpoint = Optional.of(current_rotation);
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

