package team1403.robot.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.Constants;
import team1403.robot.Constants.Swerve;

/**
 * The default command for the swerve drivetrain subsystem.
 */
public class SwerveCommand extends CommandBase {
  private final SwerveSubsystem m_drivetrainSubsystem;

  private final DoubleSupplier m_verticalTranslationSupplier;
  private final DoubleSupplier m_horizontalTranslationSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final BooleanSupplier m_fieldRelativeSupplier;
  private final DoubleSupplier m_speedSupplier;
  private boolean m_isFieldRelative;

  private Translation2d frontRight;
  private Translation2d frontLeft;
  private Translation2d backRight;
  private Translation2d backLeft;

  private SlewRateLimiter m_verticalTranslationLimiter;
  private SlewRateLimiter m_horizontalTranslationLimiter;

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
  public SwerveCommand(SwerveSubsystem drivetrain,
      DoubleSupplier horizontalTranslationSupplier,
      DoubleSupplier verticalTranslationSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldRelativeSupplier,
      DoubleSupplier speedSupplier) {
    this.m_drivetrainSubsystem = drivetrain;
    this.m_verticalTranslationSupplier = verticalTranslationSupplier;
    this.m_horizontalTranslationSupplier = horizontalTranslationSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_fieldRelativeSupplier = fieldRelativeSupplier;
    this.m_speedSupplier = speedSupplier;
    m_isFieldRelative = true;

    frontRight = new Translation2d(
        Swerve.kTrackWidth / 2.0,
        -Swerve.kWheelBase / 2.0);

    frontLeft = new Translation2d(
        Swerve.kTrackWidth / 2.0,
        Swerve.kWheelBase / 2.0);

    backRight = new Translation2d(
        -Swerve.kTrackWidth / 2.0,
        -Swerve.kWheelBase / 2.0);

    backLeft = new Translation2d(
        -Swerve.kTrackWidth / 2.0,
         Swerve.kWheelBase / 2.0);

    m_verticalTranslationLimiter = new SlewRateLimiter(8, -8, 0);
    m_horizontalTranslationLimiter = new SlewRateLimiter(8, -8, 0);

    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void execute() {
    m_drivetrainSubsystem.setSpeedLimiter(0.2 + (m_speedSupplier.getAsDouble() * 0.8));
    if (m_fieldRelativeSupplier.getAsBoolean()) {
      m_isFieldRelative = !m_isFieldRelative;
    }
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    double vertical = m_verticalTranslationLimiter.calculate(m_verticalTranslationSupplier.getAsDouble())
        * Swerve.kMaxSpeed;
    double horizontal = m_horizontalTranslationLimiter.calculate(m_horizontalTranslationSupplier.getAsDouble())
        * Swerve.kMaxSpeed;
    double angular = squareNum(m_rotationSupplier.getAsDouble()) * Swerve.kMaxAngularSpeed;
    Translation2d offset = new Translation2d();
    //double robotAngleinDegrees = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees();

    if (m_isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vertical, horizontal,
          angular, m_drivetrainSubsystem.getGyroscopeRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(vertical, horizontal, angular);
    }

    m_drivetrainSubsystem.drive(chassisSpeeds, offset);
    SmartDashboard.putBoolean("isFieldRelative", m_isFieldRelative);
  }

  private double squareNum(double num) {
    double sign = Math.signum(num);
    return sign * Math.pow(num, 2);
  }
}
