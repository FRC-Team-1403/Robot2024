package team1403.robot.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.swing.GroupLayout.Alignment;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.lib.util.CircularSlewRateLimiter;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;
import team1403.robot.Constants.Swerve;
import team1403.robot.subsystems.Blackbox;

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
  private final Supplier<Translation2d> m_targetPosSupplier;
  private final DoubleSupplier m_snipingMode;
  private final BooleanSupplier m_ampSupplier;
  private final BooleanSupplier m_alignSupplier;
  private boolean m_isFieldRelative;

  private SlewRateLimiter m_translationLimiter;
  private SlewRateLimiter m_rotationRateLimiter;
  private CircularSlewRateLimiter m_directionSlewRate;
  private static final double kDirectionSlewRateLimit = 22;


  private ProfiledPIDController m_controller;
  private TrapezoidProfile.State m_state = new TrapezoidProfile.State();
  private PPHolonomicDriveController m_driveController = new PPHolonomicDriveController(
    Constants.Swerve.kTranslationPID,
    Constants.Swerve.kRotationPID,
    Constants.Swerve.kMaxSpeed,
    Constants.Swerve.kDriveBase
  );
  private PathPlannerTrajectory.State m_driveState = new PathPlannerTrajectory.State();

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
      BooleanSupplier alignmentSupplier,
      Supplier<Translation2d> targetSupplier,
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
    this.m_targetPosSupplier = targetSupplier;
    this.m_alignSupplier = alignmentSupplier;
    m_ampSupplier = ampSupplier;
    m_snipingMode = snipingMode;
    m_isFieldRelative = true;
    //effectively no slew rate for slowing down
    m_translationLimiter = new SlewRateLimiter(2, -3, 0);
    m_rotationRateLimiter = new SlewRateLimiter(3, -3, 0);
    m_directionSlewRate = new CircularSlewRateLimiter(kDirectionSlewRateLimit);
    m_controller = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(Swerve.kMaxAngularSpeed, 80));
    m_controller.enableContinuousInput(-Math.PI, Math.PI);

    m_driveState.constraints = Constants.Swerve.kPathConstraints;

    Constants.kDriverTab.addBoolean("isFieldRelative", () -> m_isFieldRelative);
    Constants.kDebugTab.addBoolean("Aimbot", m_aimbotSupplier);
    Constants.kDebugTab.add("Swerve Rotation PID", m_controller);

    addRequirements(m_drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    m_controller.reset(MathUtil.angleModulus(m_drivetrainSubsystem.getRotation().getRadians()), 0);
  }

  @Override
  public void execute() {
    m_speedLimiter = 0.3 * (1.0 - m_snipingMode.getAsDouble() * 0.7) + (m_speedSupplier.getAsDouble() * 0.7);
  
    if (DriverStation.isAutonomousEnabled()) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds());
      return;
    }

    if (m_fieldRelativeSupplier.getAsBoolean()) {
      m_isFieldRelative = !m_isFieldRelative;
    }

    boolean x_mode = m_xModeSupplier.getAsBoolean();
    m_drivetrainSubsystem.setXModeEnabled(x_mode);
    if (x_mode) {
      m_translationLimiter.reset(0);
      return;
    }

    ChassisSpeeds chassisSpeeds;
    ChassisSpeeds currentSpeeds = m_drivetrainSubsystem.getCurrentChassisSpeed();
    double horizontal = m_horizontalTranslationSupplier.getAsDouble();
    double vertical = m_verticalTranslationSupplier.getAsDouble();

    if(CougarUtil.getAlliance() == Alliance.Red && m_isFieldRelative) {
      horizontal *= -1;
      vertical *= -1;
    }

    {
      //normalize using polar coordinates
      double vel_hypot = Math.hypot(horizontal, vertical);
      double velocity = MathUtil.clamp(vel_hypot, 0, 1);
      double angle = Math.atan2(vertical, horizontal);

      velocity *= m_speedLimiter;
      velocity = m_translationLimiter.calculate(velocity) * Swerve.kMaxSpeed;

      if(vel_hypot < 0.01) {
        angle = m_directionSlewRate.lastValue();
      }
      else if(velocity < 0.01) {
        m_directionSlewRate.setLimits(500);
        angle = m_directionSlewRate.calculate(angle);
      }
      else {
        m_directionSlewRate.setLimits(kDirectionSlewRateLimit / velocity);
        angle = m_directionSlewRate.calculate(angle);
      }

      horizontal = velocity * Math.cos(angle);
      vertical = velocity * Math.sin(angle);
    }
    double angular = m_rotationRateLimiter.calculate(squareNum(m_rotationSupplier.getAsDouble()) * m_speedLimiter) * Swerve.kMaxAngularSpeed;

    Pose2d curPose = m_drivetrainSubsystem.getPose();
    Rotation2d curRotation = curPose.getRotation();
    double given_current_angle = MathUtil.angleModulus(curRotation.getRadians());
    double given_target_angle = Math.atan2(m_targetPosSupplier.get().getY() - curPose.getY(), m_targetPosSupplier.get().getX() - curPose.getX());
    given_target_angle = MathUtil.angleModulus(given_target_angle + Math.PI);
    // double given_target_angle = Units.radiansToDegrees(Math.atan2(m_drivetrainSubsystem.getPose().getY() - m_ysupplier.getAsDouble(), m_drivetrainSubsystem.getPose().getX() - m_xsupplier.getAsDouble()));
    
    if(m_ampSupplier.getAsBoolean()) {
      given_target_angle = -Math.PI / 2;
    }

    Logger.recordOutput("Target Angle", given_target_angle);
    
    if(m_aimbotSupplier.getAsBoolean() || m_ampSupplier.getAsBoolean())
    {
      angular = m_controller.calculate(given_current_angle, given_target_angle);
      Logger.recordOutput("Aimbot Angular", angular);
    } else {
      m_state.position = given_current_angle;
      m_state.velocity = currentSpeeds.omegaRadiansPerSecond;
      m_controller.reset(m_state);
    }
    
    if(m_alignSupplier.getAsBoolean() && Blackbox.isValidTargetPosition()) {
      m_driveState.heading = Blackbox.targetPosition.getRotation();
      m_driveState.targetHolonomicRotation = Blackbox.targetPosition.getRotation();
      m_driveState.positionMeters = Blackbox.targetPosition.getTranslation();
      chassisSpeeds = m_driveController.calculateRobotRelativeSpeeds(curPose, m_driveState);
    } else {
      if (m_isFieldRelative) {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vertical, horizontal,
            angular, m_drivetrainSubsystem.getRotation());
      } else {
        chassisSpeeds = new ChassisSpeeds(vertical, horizontal, angular);
      }
      m_driveController.reset(curPose, currentSpeeds);
    }

    m_drivetrainSubsystem.drive(chassisSpeeds);
  }

  private static double squareNum(double num) {
    return Math.signum(num) * Math.pow(num, 2);
  }
}
