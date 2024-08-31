package team1403.robot.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.lib.util.CircularSlewRateLimiter;
import team1403.robot.Constants;
import team1403.robot.Constants.Swerve;

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
  private final DoubleSupplier m_snipingMode;
  private final BooleanSupplier m_ampSupplier;
  private boolean m_isFieldRelative;

  private SlewRateLimiter m_translationLimiter;
  private SlewRateLimiter m_rotationRateLimiter;
  private CircularSlewRateLimiter m_directionSlewRate;
  private static final double kDirectionSlewRateLimit = 22;


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
      BooleanSupplier ampSupplier,
      DoubleSupplier xtarget,
      DoubleSupplier ytarget,
      DoubleSupplier speedSupplier,
      DoubleSupplier snipingMode) {
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
    m_ampSupplier = ampSupplier;
    m_snipingMode = snipingMode;
    m_isFieldRelative = true;
    //effectively no slew rate for slowing down
    m_translationLimiter = new SlewRateLimiter(2, -100, 0);
    m_rotationRateLimiter = new SlewRateLimiter(3, -3, 0);
    m_directionSlewRate = new CircularSlewRateLimiter(kDirectionSlewRateLimit);
    m_controller = new PIDController(5, 0, 0);

    m_controller.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void execute() {
    m_speedLimiter = 0.3 * (1.0 - m_snipingMode.getAsDouble() * 0.9) + (m_speedSupplier.getAsDouble() * 0.7);
  
    if (m_fieldRelativeSupplier.getAsBoolean()) {
      m_isFieldRelative = !m_isFieldRelative;
    } 

    SmartDashboard.putBoolean("isFieldRelative", m_isFieldRelative);

    boolean x_mode = m_xModeSupplier.getAsBoolean();
    m_drivetrainSubsystem.setXModeEnabled(x_mode);
    if(x_mode)
      return;

    ChassisSpeeds chassisSpeeds;
    double horizontal = m_horizontalTranslationSupplier.getAsDouble();
    double vertical = m_verticalTranslationSupplier.getAsDouble();
    {
      //normalize using polar coordinates
      double velocity = MathUtil.clamp(Math.hypot(horizontal, vertical), 0, 1);
      double angle = Math.atan2(vertical, horizontal);

      velocity *= m_speedLimiter;
      velocity = m_translationLimiter.calculate(velocity) * Swerve.kMaxSpeed;

      if(velocity < 0.01) {
        m_directionSlewRate.setLimits(500);
      }
      else {
        m_directionSlewRate.setLimits(kDirectionSlewRateLimit / velocity);
      }

      angle = m_directionSlewRate.calculate(angle);

      horizontal = velocity * Math.cos(angle);
      vertical = velocity * Math.sin(angle);
    }
    double angular = m_rotationRateLimiter.calculate(squareNum(m_rotationSupplier.getAsDouble()) * m_speedLimiter) * Swerve.kMaxAngularSpeed;
    Translation2d offset = Constants.zeroTranslation;

    double given_current_angle = MathUtil.angleModulus(m_drivetrainSubsystem.getRotation().getRadians());
    double given_target_angle = Math.atan2(m_ysupplier.getAsDouble() - m_drivetrainSubsystem.getPose().getY(), m_xsupplier.getAsDouble() - m_drivetrainSubsystem.getPose().getX());
    given_target_angle = MathUtil.angleModulus(given_target_angle + Math.PI);
    // double given_target_angle = Units.radiansToDegrees(Math.atan2(m_drivetrainSubsystem.getPose().getY() - m_ysupplier.getAsDouble(), m_drivetrainSubsystem.getPose().getX() - m_xsupplier.getAsDouble()));
    
    if(m_ampSupplier.getAsBoolean()) {
      if(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue)
        given_target_angle = -Math.PI/2;
      else
        given_target_angle = Math.PI/2;
    }

    Logger.recordOutput("Target Angle", given_target_angle);
    SmartDashboard.putBoolean("Aimbot", m_aimbotSupplier.getAsBoolean());
    
    if(m_aimbotSupplier.getAsBoolean() || m_ampSupplier.getAsBoolean())
    {
      angular = m_controller.calculate(given_current_angle, given_target_angle);
      Logger.recordOutput("Aimbot Angular", angular);
    }
    
    if (m_isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vertical, horizontal,
          angular, m_drivetrainSubsystem.getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(vertical, horizontal, angular);
    }

    m_drivetrainSubsystem.drive(chassisSpeeds, offset);
  }

  private static double squareNum(double num) {
    return Math.signum(num) * Math.pow(num, 2);
  }
}
