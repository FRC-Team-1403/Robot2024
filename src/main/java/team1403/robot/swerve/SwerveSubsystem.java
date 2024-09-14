package team1403.robot.swerve;

import java.util.ArrayList;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.NavxAhrs;
import team1403.robot.Constants;
import team1403.robot.Constants.CanBus;
import team1403.robot.Constants.Swerve;

/**
 * The drivetrain of the robot. Consists of for swerve modules and the
 * gyroscope.
 */
public class SwerveSubsystem extends SubsystemBase {
  private final NavxAhrs m_navx2;
  private final SwerveModule[] m_modules;
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private SwerveModuleState[] m_currentStates = new SwerveModuleState[4];
  private SwerveModulePosition[] m_currentPositions = new SwerveModulePosition[4];
  private final SyncSwerveDrivePoseEstimator m_odometer;
  private Field2d m_field = new Field2d();
  // private double m_speedLimiter = 0.6;

  private Translation2d m_offset;

  private double m_rollOffset;

  private boolean m_isXModeEnabled = false;
  private ArrayList<AprilTagCamera> m_cameras;
  private boolean m_disableVision = false;
  private boolean m_rotDriftCorrect = false;
  private SwerveHeadingCorrector m_headingCorrector = new SwerveHeadingCorrector();

  private static final SwerveModuleState[] m_xModeState = {
    // Front Left
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    // Front Right
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    // Back left
    new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
    // Back Right
    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
  };

  private final Notifier m_odometeryNotifier;

  /**
   * Creates a new {@link SwerveSubsystem}.
   * Instantiates the 4 {@link SwerveModule}s,
   * the {@link SwerveDriveOdometry}, and the {@link NavxAhrs}.
   * Also sets drivetrain ramp rate,
   * and idle mode to default values.
   *
   * @param parameters the {@link CougarLibInjectedParameters}
   *                   used to construct this subsystem
   */
  public SwerveSubsystem() {
    SmartDashboard.putData("Field", m_field);
    // super("Swerve Subsystem", parameters);
    m_navx2 = new NavxAhrs("Gyroscope", SerialPort.Port.kMXP);
    m_modules = new SwerveModule[] {
        new SwerveModule("Front Left Module",
            CanBus.frontLeftDriveID, CanBus.frontLeftSteerID,
            CanBus.frontLeftEncoderID, Swerve.frontLeftEncoderOffset),
        new SwerveModule("Front Right Module",
            CanBus.frontRightDriveID, CanBus.frontRightSteerID,
            CanBus.frontRightEncoderID, Swerve.frontRightEncoderOffset),
        new SwerveModule("Back Left Module",
            CanBus.backLeftDriveID, CanBus.backLeftSteerID,
            CanBus.backLeftEncoderID, Swerve.backLeftEncoderOffset),
        new SwerveModule("Back Right Module",
            CanBus.backRightDriveID, CanBus.backRightSteerID,
            CanBus.backRightEncoderID, Swerve.backRightEncoderOffset),
    };

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveNoOffset, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            Swerve.kTranslationPID, // Translation PID
                                                                                                // constants
            Swerve.kRotationPID, // Rotation PID
                                                                                                // constants
            Swerve.kMaxSpeed, // Max module speed, in m/s
            Swerve.kDriveBase, // Drive base radius in meters. Distance from
                                                                           // robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),

        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          return Constants.kAllianceSupplier.get() == DriverStation.Alliance.Red;
        },
        this // Reference to this subsystem to set requirements
    );
    Pathfinding.setPathfinder(new LocalADStar());
    PathfindingCommand.warmupCommand().schedule();
    PathPlannerLogging.setLogActivePathCallback((activePath) -> {
      Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    });
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
        Logger.recordOutput("Odometery/TrajectorySetpoint", targetPose);
    });

    // addDevice(m_navx2.getName(), m_navx2);
    if (m_navx2.isConnected())
      while (m_navx2.isCalibrating());

    zeroGyroscope();

    m_odometer = new SyncSwerveDrivePoseEstimator(new Pose2d(), () -> getGyroscopeRotation(), () -> getModulePositions());

    setRobotRampRate(0.0);
    setRobotIdleMode(IdleMode.kBrake);

    m_offset = new Translation2d();
    m_rollOffset = -m_navx2.getRoll();

    m_cameras = new ArrayList<>();
    m_cameras.add(new AprilTagCamera("Unknown_Camera", () -> Swerve.kCameraTransfrom, this::getPose));

    m_odometeryNotifier = new Notifier(this::highFreqUpdate);
    m_odometeryNotifier.setName("SwerveOdoNotifer");
    m_odometeryNotifier.startPeriodic(Units.millisecondsToSeconds(Constants.kSwerveModuleUpdateRateMs));
  }

  public void setDisableVision(boolean disable) {
    m_disableVision = disable;
  }

  /**
   * Gets the 4 swerve module positions.
   *
   * @return an array of swerve module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    for(int i = 0; i < m_modules.length; i++) {
      m_currentPositions[i] = m_modules[i].getModulePosition();
    }
    return m_currentPositions;
  }

  /**
   * Sets the ramp rate of the drive motors.
   *
   * @param rate the ramp rate
   */
  public void setRobotRampRate(double rate) {
    for (SwerveModule module : m_modules) {
      module.setRampRate(rate);
    }
  }

  /**
   * Sets the idle mode for the drivetrain.
   *
   * @param mode the IdleMode of the robot
   */
  public void setRobotIdleMode(IdleMode mode) {
    for (SwerveModule module : m_modules) {
      module.setControllerMode(mode);
    }
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' directi=on.
   */
  private void zeroGyroscope() {
    // tracef("zeroGyroscope %f", getGyroscopeRotation());
    m_navx2.reset();
  }

  public void zeroHeading() {
    zeroGyroscope();
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    m_headingCorrector.resetHeadingSetpoint();
  }

  /**
   * Return the position of the drivetrain.
   *
   * @return the position of the drivetrain in Pose2d
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_odometer.getPose();
  }

  /**
   * Reset the position of the drivetrain odometry.
   */
  public void resetOdometry() {
    resetOdometry(getPose());
  }

  /**
   * Reset the position of the drivetrain odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometer.resetPosition(pose);
  }

  /**
   * Gets the heading of the gyroscope.
   *
   * @return a Rotation2d object that contains the gyroscope's heading
   */
  private Rotation2d getGyroscopeRotation() {
    return m_navx2.getRotation2d();
  }

  /**
   * Gets the heading of the robot.
   *
   * @return a Rotation2d object that contains the robot's heading
   */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Gets the roll of the gyro (Y axis of gyro rotation).
   * 
   * @return a double representing the roll of robot in degrees
   */
  public double getGyroRoll() {
    return m_navx2.getRoll() + m_rollOffset;
  }

  /**
   * Gets the pitch of the gyro (X axis of gyro rotation).
   * 
   * @return a double representing the pitch of robot in degrees
   */
  public double getGyroPitch() {
    return m_navx2.getPitch();
  }

  /**
   * Moves the drivetrain at the given chassis speeds.
   *
   * @param chassisSpeeds the speed to move at
   * @param offset        the swerve module to pivot around
   */
  public void drive(ChassisSpeeds chassisSpeeds, Translation2d offset) {
    m_chassisSpeeds = translationalDriftCorrection(chassisSpeeds);
    m_offset = offset;
  }

  public void driveNoOffset(ChassisSpeeds chassisSpeeds) {
    drive(chassisSpeeds, Constants.zeroTranslation);
  }

  /**
   * Stops the drivetrain.
   */
  public void stop() {
    m_chassisSpeeds = new ChassisSpeeds();
  }

  /**
   * Sets the module speed and heading for all 4 modules.
   *
   * @param states an array of states for each module.
   */
  
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, Swerve.kMaxSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      states[i] = SwerveModuleState.optimize(states[i], new Rotation2d(m_modules[i].getAbsoluteAngle()));
      m_modules[i].set(states[i].speedMetersPerSecond,
          MathUtil.angleModulus(states[i].angle.getRadians()));
    }

    Logger.recordOutput("SwerveStates/Target", states);
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    for(int i = 0; i < m_modules.length; i++) {
      m_currentStates[i] = m_modules[i].getState();
    }
    return m_currentStates;
  }

  public ChassisSpeeds getTargetChassisSpeed() {
    return m_chassisSpeeds;
  }

  public ChassisSpeeds getCurrentChassisSpeed() {
    return Swerve.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Resets the robot to no longer pivot around one wheel.
   */
  public void setMiddlePivot() {
    m_offset = new Translation2d();
  }

  /**
   * Puts the drivetrain into xMode where all the wheel put towards the center of
   * the robot, 
   * making it harder for the robot to be pushed around.
   */
  private void xMode() {
    setModuleStates(m_xModeState);
  }

  /**
   * Sets the drivetain in xMode.
   * 
   * @param enabled whether the drivetrain is in xMode.
   */
  public void setXModeEnabled(boolean enabled) {
    this.m_isXModeEnabled = enabled;
  }

  /**
   * Accounts for the drift caused by the first order kinematics
   * while doing both translational and rotational movement.
   * 
   * <p>
   * Looks forward one control loop to figure out where the robot
   * should be given the chassisspeed and backs out a twist command from that.
   * 
   * @param chassisSpeeds the given chassisspeeds
   * @return the corrected chassisspeeds
   */
  private ChassisSpeeds translationalDriftCorrection(ChassisSpeeds chassisSpeeds) {
    return ChassisSpeeds.discretize(chassisSpeeds, Constants.kLoopTime);
  }

  public void setEnableRotDriftCorrect(boolean state) {
    m_rotDriftCorrect = state;
    m_headingCorrector.resetHeadingSetpoint();
  }

  private ChassisSpeeds rotationalDriftCorrection(ChassisSpeeds speeds) {
    ChassisSpeeds corrected = m_headingCorrector.update(speeds, getCurrentChassisSpeed(), getRotation(), m_navx2.getAngularVelocity());
    if (m_rotDriftCorrect && !DriverStation.isAutonomousEnabled())
    {
      return corrected;
    }

    return speeds;
  }

  private void highFreqUpdate() {
    m_odometer.update();
  }

  @Override
  public void periodic() {
    if(!m_disableVision)
    {
      for(AprilTagCamera cam : m_cameras)
      {
        if (cam.hasTarget() && cam.hasPose()) {
          Pose2d pose = cam.getPose2D();
          if (pose != null && cam.checkVisionResult()) {
            m_odometer.addVisionMeasurement(pose, cam.getTimestamp(), cam.getEstStdv());
          }
        }
      }
    }
    // SmartDashboard.putNumber("Speed", m_speedLimiter);

    if (this.m_isXModeEnabled) {
      xMode();
    } else {
      ChassisSpeeds corrected = rotationalDriftCorrection(m_chassisSpeeds);
      
      setModuleStates(Swerve.kDriveKinematics.toSwerveModuleStates(corrected, m_offset));
    }
    m_field.setRobotPose(getPose());
    // Logging Output
    Logger.recordOutput("Gyro Roll", getGyroRoll());

    Logger.recordOutput("Chassis Speeds", getCurrentChassisSpeed().toString());

    // Logger.recordOutput("Front Left Absolute Encoder Angle", m_modules[0].getAbsoluteAngle());
    // Logger.recordOutput("Front Right Absolute Encoder Angle", m_modules[1].getAbsoluteAngle());
    // Logger.recordOutput("Back Left Absolute Encoder Angle", m_modules[2].getAbsoluteAngle());
    // Logger.recordOutput("Back Right Absolute Encoder Angle", m_modules[3].getAbsoluteAngle());

    Logger.recordOutput("Gyro Reading", getGyroscopeRotation().getDegrees());
  }
}
