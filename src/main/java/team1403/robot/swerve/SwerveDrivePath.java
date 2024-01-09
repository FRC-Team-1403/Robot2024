package team1403.robot.swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team1403.robot.Constants;
import team1403.robot.Constants.Swerve;

/**
 * Autonomous command to be used with the CSE.
 */
public class SwerveDrivePath extends CommandBase {

  private SwerveSubsystem m_drivetrain;

  private List<Translation2d> m_wayPoints;
  private final Pose2d m_startPose;
  private final Pose2d m_endPose;

  private TrajectoryConfig m_trajectoryConfig;
  private Trajectory m_trajectory;
  private PIDController m_verticalTranslationController;
  private PIDController m_horizontalTranslationController;
  private ProfiledPIDController m_angleController;
  private SwerveControllerCommand m_swerveControllerCommand;

  /**
   * Constructs the path to drive.
   *
   * @param drivetrain the instance of the drivetrain subsystem
   * @param startAngle the starting angle of the robot
   * @param endAngle the ending angle of the robot
   * @param wayPoints list of wapoints for the robot to move through
   */
  public SwerveDrivePath(SwerveSubsystem drivetrain, double startAngle, double endAngle,
      List<Translation2d> wayPoints) {
    this.m_drivetrain = drivetrain;
    this.m_wayPoints = wayPoints;

    for (int i = 0; i < wayPoints.size(); i++) {
      double x = wayPoints.get(i).getX();
      double y = wayPoints.get(i).getY();
      wayPoints.set(i, wayPoints.get(i).plus(
          new Translation2d(-x + y, -y + x)));
      wayPoints.set(i, wayPoints.get(i).times(0.30478512648));
    }

    m_startPose = new Pose2d(wayPoints.get(0).getX(), wayPoints.get(0).getY(), 
        Rotation2d.fromDegrees(startAngle));
    m_endPose = new Pose2d(wayPoints.get(wayPoints.size() - 1).getX(), 
        wayPoints.get(wayPoints.size() - 1).getY(),
        Rotation2d.fromDegrees(endAngle));

    wayPoints.remove(0);
    wayPoints.remove(wayPoints.size() - 1);

    m_trajectoryConfig = new TrajectoryConfig(
        Swerve.kMaxSpeed / 10,
        0.5)
        .setKinematics(Swerve.kDriveKinematics);

    m_verticalTranslationController = new PIDController(Swerve.kPTranslation, 
        Swerve.kITranslation, Swerve.kDTranslation);
    m_horizontalTranslationController = new PIDController(Swerve.kPTranslation, 
        Swerve.kITranslation, Swerve.kDTranslation);

    m_angleController = new ProfiledPIDController(
        Swerve.kPAutoTurning, Swerve.kIAutoTurning, 
        Swerve.kDAutoTurning, Swerve.kThetaControllerConstraints);
  }

  @Override
  public void initialize() {
    m_drivetrain.zeroGyroscope();

    m_drivetrain.increaseSpeed(1);

    m_trajectory = TrajectoryGenerator.generateTrajectory(
        m_startPose,
        m_wayPoints,
        m_endPose,
        m_trajectoryConfig);

    m_swerveControllerCommand = new SwerveControllerCommand(
        m_trajectory,
        m_drivetrain::getPose,
        m_horizontalTranslationController,
        m_verticalTranslationController,
        m_angleController,
        m_drivetrain);

    m_swerveControllerCommand.schedule();
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return m_swerveControllerCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setSpeedLimiter(0.6);
    m_drivetrain.stop();
  }
}
