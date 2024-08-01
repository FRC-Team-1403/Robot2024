package team1403.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import team1403.lib.util.TimeDelayedBoolean;

//adapted from team 254
public class SwerveHeadingCorrector {

    private TimeDelayedBoolean zero_yaw_detector = new TimeDelayedBoolean();
    //initial rotation should be 0
    private double yaw_setpoint = 0;
    private PIDController m_controller = new PIDController(4, 0, 0);


    public SwerveHeadingCorrector()
    {
        m_controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ChassisSpeeds update(double timestamp, ChassisSpeeds target, ChassisSpeeds cur_vel, Rotation2d gyro)
    {
        double current_rotation = MathUtil.angleModulus(gyro.getRadians());
        boolean near_zero_ang_z = zero_yaw_detector.update(Math.abs(cur_vel.omegaRadiansPerSecond) < 0.03, 0.15);
        boolean is_translating = Math.hypot(target.vxMetersPerSecond, target.vyMetersPerSecond) > 0.1;
        
        if(near_zero_ang_z || Math.abs(target.omegaRadiansPerSecond) > 0.03)
        {
            yaw_setpoint = current_rotation;
        }


        if(is_translating && Math.abs(target.omegaRadiansPerSecond) < 0.03)
        {
            double new_yaw = m_controller.calculate(current_rotation, yaw_setpoint);

            return new ChassisSpeeds(target.vxMetersPerSecond, target.vyMetersPerSecond, new_yaw);
        }



        return target;
    }
}
