package team1403.robot.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ejml.ops.FConvertArrays;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import team1403.robot.Constants;
import team1403.robot.Constants.Swerve;
import team1403.robot.Constants.Vision;

/**
 * The default command for the swerve drivetrain subsystem.
 */
public class DefaultSwerveCommand extends Command {
  private final SwerveSubsystem m_drivetrainSubsystem;

  private final DoubleSupplier m_verticalTranslationSupplier;
  private final DoubleSupplier m_horizontalTranslationSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final BooleanSupplier m_fieldRelativeSupplier;
  private final BooleanSupplier m_xModeSupplier;
  private final BooleanSupplier m_aimbotSupplier;
  private final DoubleSupplier m_speedSupplier;
  private final DoubleSupplier m_xsupplier;
  private final DoubleSupplier m_ysupplier;
  private boolean m_isFieldRelative;
  private double tempKP = 0;
  private double tempKI = 0;

  private SlewRateLimiter m_verticalTranslationLimiter;
  private SlewRateLimiter m_horizontalTranslationLimiter;

  private PIDController m_controller;

  private double m_speedLimiter = 0.2;

  /**
   * Creates the swerve command.
   * \
   * 
   * @param drivetrain                    the instance of the
   *                                      {@link SwerveSubsystem}
   * @param horizontalTranslationSupplier
   *                                      supplies the horizontal speed of the
   *                                      drivetrain
   * @param verticalTranslationSupplier
   *                                      supplies the the vertical speed of the
   *                                      drivetrain
   * @param rotationSupplier              supplies the rotational speed of the
   *                                      drivetrain
   * @param fieldRelativeSupplier         supplies the
   *                                      boolean value to enable field relative
   *                                      mode
   */
  public DefaultSwerveCommand(SwerveSubsystem drivetrain,
      DoubleSupplier horizontalTranslationSupplier,
      DoubleSupplier verticalTranslationSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldRelativeSupplier,
      BooleanSupplier xModeSupplier,
      BooleanSupplier aimbotSupplier,
      DoubleSupplier xtarget,
      DoubleSupplier ytarget,
      DoubleSupplier speedSupplier) {
    this.m_drivetrainSubsystem = drivetrain;
    this.m_verticalTranslationSupplier = verticalTranslationSupplier;
    this.m_horizontalTranslationSupplier = horizontalTranslationSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_fieldRelativeSupplier = fieldRelativeSupplier;
    this.m_speedSupplier = speedSupplier;
    this.m_xModeSupplier = xModeSupplier;
    this.m_aimbotSupplier = aimbotSupplier;
    this.m_xsupplier = xtarget;
    this.m_ysupplier = ytarget;
    m_isFieldRelative = true;
    m_verticalTranslationLimiter = new SlewRateLimiter(8, -8, 0);
    m_horizontalTranslationLimiter = new SlewRateLimiter(8, -8, 0);
    m_controller = new PIDController(1.25, 1, 0);

    addRequirements(m_drivetrainSubsystem);

    SmartDashboard.putNumber("P Value", tempKP);
    SmartDashboard.putNumber("I Value", tempKI);
  }

  @Override
  public void execute() {
    tempKP = SmartDashboard.getNumber("P Value", tempKP);
    tempKI = SmartDashboard.getNumber("I Value", tempKI);
    m_controller.setP(tempKP);
    m_controller.setI(tempKI);

    m_speedLimiter = 0.2 + (m_speedSupplier.getAsDouble() * 0.8);

    if (m_fieldRelativeSupplier.getAsBoolean()) {
      m_isFieldRelative = !m_isFieldRelative;
    } 

    SmartDashboard.putBoolean("isFieldRelative", m_isFieldRelative);

    {
      boolean x_mode = m_xModeSupplier.getAsBoolean();
      m_drivetrainSubsystem.setXModeEnabled(x_mode);
      if(x_mode)
        return;
    }

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    double vertical = m_verticalTranslationLimiter.calculate(m_verticalTranslationSupplier.getAsDouble())
        * Swerve.kMaxSpeed * m_speedLimiter;
    double horizontal = m_horizontalTranslationLimiter.calculate(m_horizontalTranslationSupplier.getAsDouble())
        * Swerve.kMaxSpeed * m_speedLimiter;
    double angular = squareNum(m_rotationSupplier.getAsDouble()) * Swerve.kMaxAngularSpeed * m_speedLimiter;
    Translation2d offset = new Translation2d();

    
    double given_current_angle = m_drivetrainSubsystem.getNavxAhrs().getRotation2d().getDegrees();
    double given_target_angle = Units.radiansToDegrees(Math.atan2(m_ysupplier.getAsDouble() - m_drivetrainSubsystem.getPose().getY(), m_xsupplier.getAsDouble() - m_drivetrainSubsystem.getPose().getX()));
    // double given_target_angle = Units.radiansToDegrees(Math.atan2(m_drivetrainSubsystem.getPose().getY() - m_ysupplier.getAsDouble(), m_drivetrainSubsystem.getPose().getX() - m_xsupplier.getAsDouble()));
    double constraint_current_angle = GetConstraintAngle(given_current_angle);
    double final_target_angle = 0;

    if (constraint_current_angle < 0)
      final_target_angle = GetFinalTargetAngleForNegativeCurrentAngle(constraint_current_angle, given_current_angle, given_target_angle);
    else if(constraint_current_angle > 0 && given_target_angle > 0 && constraint_current_angle > given_target_angle)
      final_target_angle = -(constraint_current_angle - given_target_angle);
    else if(constraint_current_angle > 0 && given_current_angle > 0 && constraint_current_angle < given_current_angle)
      final_target_angle = ((180 - given_current_angle) - constraint_current_angle);
    else if(constraint_current_angle > 0 && given_current_angle > 0 && constraint_current_angle > given_current_angle)
      final_target_angle = -((180 - given_current_angle) - given_target_angle); 
    else 
      final_target_angle = (given_target_angle - constraint_current_angle);

    
    m_drivetrainSubsystem.setDisableVision(m_aimbotSupplier.getAsBoolean());
    SmartDashboard.putNumber("Target Angle",final_target_angle);
    

    if(m_aimbotSupplier.getAsBoolean() && Math.abs(final_target_angle) > 0)
      angular = m_controller.calculate(0, final_target_angle);
      // angular = m_controller.calculate(robotAngleinDegrees, target_angle);
      //angular = m_controller.calculate(sub2,0);

    if (m_isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vertical, horizontal,
          angular, m_drivetrainSubsystem.getGyroscopeRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(vertical, horizontal, angular);
    }

    m_drivetrainSubsystem.drive(chassisSpeeds, offset);
  }
       
  public static double GetConstraintAngle(double angle) {
      while(angle > 180)
        angle -= 360;
      while(angle < -180)
       angle += 360;
      return angle;
  }
   
    public static double GetFinalTargetAngleForNegativeCurrentAngle(double constraint_current_angle,
    double given_current_angle, double given_target_angle) {
      double calculated_current_angle = constraint_current_angle;
        
      if(constraint_current_angle*-1 + given_target_angle > 180)
        calculated_current_angle = 360 + constraint_current_angle;
           
      double calculated_target_angle = calculated_current_angle - given_target_angle;
       
      double final_target_angle = -calculated_target_angle;
      return final_target_angle;
    }


  private double squareNum(double num) {
    double sign = Math.signum(num);
    return sign * Math.pow(num, 2);
  }
}
