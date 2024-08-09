package team1403.robot.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.lib.util.ContinuousSlewRateLimiter;
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
  private BooleanSupplier m_snipingMode;
  private boolean m_isFieldRelative;
  private double tempKP = 0;
  private double tempKI = 0;

  private SlewRateLimiter m_translationLimiter;
  private SlewRateLimiter m_rotationRateLimiter;

  private ProfiledPIDController m_controller;

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
      DoubleSupplier speedSupplier,
      BooleanSupplier snipingMode) {
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
    m_snipingMode = snipingMode;
    m_isFieldRelative = true;
    m_translationLimiter = new SlewRateLimiter(3);
    m_rotationRateLimiter = new SlewRateLimiter(4);
    m_controller = new ProfiledPIDController(7.5, 0, 0, new TrapezoidProfile.Constraints(Swerve.kMaxAngularSpeed / 4, 30));

    m_controller.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_drivetrainSubsystem);

    SmartDashboard.putNumber("P Value", tempKP);
    SmartDashboard.putNumber("I Value", tempKI);
  }

  @Override
  public void execute() {


    m_speedLimiter = 0.3 + (m_speedSupplier.getAsDouble() * 0.7);
    if (m_snipingMode.getAsBoolean()) {
      m_speedLimiter = (m_speedSupplier.getAsDouble() * 0.25) + 0.05;
    }
    if (m_fieldRelativeSupplier.getAsBoolean()) {
      m_isFieldRelative = !m_isFieldRelative;
    } 

    SmartDashboard.putBoolean("isFieldRelative", m_isFieldRelative);

      boolean x_mode = m_xModeSupplier.getAsBoolean();
      m_drivetrainSubsystem.setXModeEnabled(x_mode);
      if(x_mode)
        return;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    double horizontal = m_horizontalTranslationSupplier.getAsDouble() * m_speedLimiter;
    double vertical = m_verticalTranslationSupplier.getAsDouble() * m_speedLimiter;
    double angular = m_rotationRateLimiter.calculate(squareNum(m_rotationSupplier.getAsDouble()) * m_speedLimiter) * Swerve.kMaxAngularSpeed;
    {
      //normalize using polar coordinates
      double velocity = MathUtil.clamp(Math.hypot(horizontal, vertical), 0, 1);
      double angle = Math.atan2(vertical, horizontal);

      velocity = m_translationLimiter.calculate(velocity) * Swerve.kMaxSpeed;

      horizontal = velocity * Math.cos(angle);
      vertical = velocity * Math.sin(angle);
    }

    Translation2d offset = new Translation2d();

    double given_current_angle = MathUtil.angleModulus(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
    double given_target_angle = Math.atan2(m_ysupplier.getAsDouble() - m_drivetrainSubsystem.getPose().getY(), m_xsupplier.getAsDouble() - m_drivetrainSubsystem.getPose().getX());
    given_target_angle = MathUtil.angleModulus(given_target_angle + Math.PI);
    // double given_target_angle = Units.radiansToDegrees(Math.atan2(m_drivetrainSubsystem.getPose().getY() - m_ysupplier.getAsDouble(), m_drivetrainSubsystem.getPose().getX() - m_xsupplier.getAsDouble()));
    SmartDashboard.putNumber("Target Angle", given_target_angle);
    SmartDashboard.putBoolean("Aimbot", m_aimbotSupplier.getAsBoolean());
    SmartDashboard.putNumber("Current Angle", given_current_angle);
    
    if(m_aimbotSupplier.getAsBoolean())
      angular = m_controller.calculate(given_current_angle, given_target_angle);
    SmartDashboard.putNumber("Aimbot Movement", angular);
    
    if (m_isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vertical, horizontal,
          angular, m_drivetrainSubsystem.getGyroscopeRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(vertical, horizontal, angular);
    }

    m_drivetrainSubsystem.drive(chassisSpeeds, offset);
  }

  private static double squareNum(double num) {
    return Math.signum(num) * Math.pow(num, 2);
  }
}
