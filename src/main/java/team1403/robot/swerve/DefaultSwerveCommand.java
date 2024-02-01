package team1403.robot.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
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

  private SlewRateLimiter m_verticalTranslationLimiter;
  private SlewRateLimiter m_horizontalTranslationLimiter;

  private PIDController m_controller;

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
    m_controller = new PIDController(1, 0, 0);

    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void execute() {
    m_drivetrainSubsystem.setSpeedLimiter(0.2 + (m_speedSupplier.getAsDouble() * 0.8));

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
        * Swerve.kMaxSpeed;
    double horizontal = m_horizontalTranslationLimiter.calculate(m_horizontalTranslationSupplier.getAsDouble())
        * Swerve.kMaxSpeed;
    double angular = squareNum(m_rotationSupplier.getAsDouble()) * Swerve.kMaxAngularSpeed;
    Translation2d offset = new Translation2d();
    double robotAngleinDegrees = m_drivetrainSubsystem.getNavxAhrs().get0to360Rotation2d().getDegrees();

    double target_angle = Units.radiansToDegrees(Math.atan2(m_drivetrainSubsystem.getPose().getY() - m_ysupplier.getAsDouble(), m_drivetrainSubsystem.getPose().getX() - m_xsupplier.getAsDouble()));

    //target_angle = robotAngleinDegrees + to_deg(atan2(y2 - y1, x2 - x1));

    // double sub = 0;


    if(target_angle > robotAngleinDegrees && target_angle - robotAngleinDegrees > 180) target_angle = - ((360 - target_angle) + robotAngleinDegrees);
    else if(robotAngleinDegrees > target_angle && robotAngleinDegrees - target_angle > 180) target_angle = (360 - robotAngleinDegrees) + target_angle;
    else if(target_angle > robotAngleinDegrees && target_angle - robotAngleinDegrees < 180) target_angle = target_angle - robotAngleinDegrees;
    else if(robotAngleinDegrees > target_angle && robotAngleinDegrees - target_angle < 180) target_angle = -(robotAngleinDegrees - target_angle);
    else if(target_angle == robotAngleinDegrees) target_angle = 0;
    else if(target_angle - robotAngleinDegrees == 180 || robotAngleinDegrees - target_angle == 180) target_angle = 180;
    else if(target_angle - robotAngleinDegrees == 0 || robotAngleinDegrees - target_angle == 0) target_angle = 0;



    // if(Math.abs(robotAngleinDegrees - target_angle) > 180)
    //   sub = 180;

    //double sub2 = target_angle - robotAngleinDegrees;


    m_drivetrainSubsystem.setDisableVision(m_aimbotSupplier.getAsBoolean());
    SmartDashboard.putNumber("Target Angle", target_angle);

    if(m_aimbotSupplier.getAsBoolean() && Math.abs(Math.abs(target_angle) - Math.abs(robotAngleinDegrees)) > Vision.rotationCutoff)

      angular = m_controller.calculate(target_angle,0);
      // angular = m_controller.calculate(robotAngleinDegrees, target_angle);
      //angular = m_controller.calculate(sub2,0);

    if (m_isFieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vertical, horizontal,
          angular, m_drivetrainSubsystem.getGyroscopeRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(vertical, horizontal, angular);
    }

    SmartDashboard.putString("pose2d", m_drivetrainSubsystem.getPose().toString());

    m_drivetrainSubsystem.drive(chassisSpeeds, offset);
  }

  private double squareNum(double num) {
    double sign = Math.signum(num);
    return sign * Math.pow(num, 2);
  }
}
