package team1403.robot.swerve;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;
import team1403.lib.device.wpi.NavxAhrs;
import team1403.lib.util.CougarLogged;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;
import team1403.robot.Robot;
import team1403.robot.Constants.CanBus;
import team1403.robot.Constants.Swerve;

/**
 * The drivetrain of the robot. Consists of for swerve modules and the
 * gyroscope.
 */
public class SwerveSubsystem extends SubsystemBase implements CougarLogged {
  private final NavxAhrs m_navx2;
  private final ISwerveModule[] m_modules;
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private final SwerveModuleState[] m_currentStates = new SwerveModuleState[4];
  private final SwerveModulePosition[] m_currentPositions = new SwerveModulePosition[4];
  private final SyncSwerveDrivePoseEstimator m_odometer;
  private final Field2d m_field = new Field2d();

  private boolean m_isXModeEnabled = false;
  private final ArrayList<AprilTagCamera> m_cameras = new ArrayList<>();
  private boolean m_disableVision = false;
  private boolean m_rotDriftCorrect = true;
  private final SwerveHeadingCorrector m_headingCorrector = new SwerveHeadingCorrector();
  private SimDouble m_gryoHeadingSim;
  private SimDouble m_gyroRateSim;

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
    // increase update rate because of async odometery (60 hz to 100 hz)
    m_navx2 = new NavxAhrs("Gyroscope", SerialPort.Port.kMXP, (byte)100);
    if(Robot.isReal()) {
      m_modules = new ISwerveModule[] {
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
    } else {
      m_modules = new ISwerveModule[] {
        new SimSwerveModule("Front Left Module"),
        new SimSwerveModule("Front Right Module"),
        new SimSwerveModule("Back Left Module"),
        new SimSwerveModule("Back Right Module")
      };

      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      m_gryoHeadingSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
      m_gyroRateSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Rate"));
    }

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
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
        CougarUtil::shouldMirrorPath,
        this // Reference to this subsystem to set requirements
    );
    Pathfinding.setPathfinder(new LocalADStar());
    PathfindingCommand.warmupCommand().schedule();
    PathPlannerLogging.setLogActivePathCallback((activePath) -> {
      log("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
      m_field.getObject("traj").setPoses(activePath);
    });
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
      log("Odometry/TrajectorySetpoint", targetPose);
    });

    // addDevice(m_navx2.getName(), m_navx2);
    if (m_navx2.isConnected())
      while (m_navx2.isCalibrating());

    zeroGyroscope();

    m_odometer = new SyncSwerveDrivePoseEstimator(new Pose2d(), () -> getGyroscopeRotation(), () -> getModulePositions());

    VisionSimUtil.initVisionSim();

    m_cameras.add(new AprilTagCamera("Unknown_Camera", () -> Swerve.kCameraTransfrom, this::getPose));

    m_odometeryNotifier = new Notifier(this::highFreqUpdate);
    m_odometeryNotifier.setName("SwerveOdoNotifer");
    m_odometeryNotifier.startPeriodic(Units.millisecondsToSeconds(Constants.Swerve.kModuleUpdateRateMs));

    Constants.kDriverTab.add("Gyro", m_navx2);
    Constants.kDriverTab.add("Field", m_field);
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
    if(CougarUtil.getAlliance() == Alliance.Blue)
      resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    else
      resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d(Math.PI)));
    m_headingCorrector.resetHeadingSetpoint();
  }

  /**
   * Return the position of the drivetrain.
   *
   * @return the position of the drivetrain in Pose2d
   */
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
   * Sets the target chassis speeds
   * @param chassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = translationalDriftCorrection(chassisSpeeds);
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
    SwerveModuleState[] currentStates = getModuleStates();

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.kMaxSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      states[i] = SwerveModuleState.optimize(states[i], currentStates[i].angle);
      m_modules[i].set(states[i].speedMetersPerSecond,
          MathUtil.angleModulus(states[i].angle.getRadians()));
    }

    log("SwerveStates/Target", states);
  }


  public SwerveModuleState[] getModuleStates() {
    for(int i = 0; i < m_modules.length; i++) {
      m_currentStates[i] = m_modules[i].getState();
    }
    log("SwerveStates/Measured", m_currentStates);
    return m_currentStates;
  }

  public ChassisSpeeds getTargetChassisSpeed() {
    return m_chassisSpeeds;
  }

  public ChassisSpeeds getCurrentChassisSpeed() {
    ChassisSpeeds ret =  Swerve.kDriveKinematics.toChassisSpeeds(getModuleStates());
    log("SwerveStates/Current Chassis Speeds", ret);
    return ret;
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
  public void simulationPeriodic() {
    VisionSimUtil.update(getPose());
    double vel = getCurrentChassisSpeed().omegaRadiansPerSecond;
    m_gyroRateSim.set(Units.radiansToDegrees(-vel));
    m_gryoHeadingSim.set(m_gryoHeadingSim.get() - Units.radiansToDegrees(vel) * Constants.kLoopTime);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty("Front Left Angle", () -> m_currentStates[0].angle.getRadians(), null);
    builder.addDoubleProperty("Front Left Velocity", () -> m_currentStates[0].speedMetersPerSecond, null);

    builder.addDoubleProperty("Front Right Angle", () -> m_currentStates[1].angle.getRadians(), null);
    builder.addDoubleProperty("Front Right Velocity", () -> m_currentStates[1].speedMetersPerSecond, null);

    builder.addDoubleProperty("Back Left Angle", () -> m_currentStates[2].angle.getRadians(), null);
    builder.addDoubleProperty("Back Left Velocity", () -> m_currentStates[2].speedMetersPerSecond, null);

    builder.addDoubleProperty("Back Right Angle", () -> m_currentStates[3].angle.getRadians(), null);
    builder.addDoubleProperty("Back Right Velocity", () -> m_currentStates[3].speedMetersPerSecond, null);

    builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
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

      log("SwerveStates/Corrected Target Chassis Speeds", corrected);

      setModuleStates(Swerve.kDriveKinematics.toSwerveModuleStates(corrected));
    }
    m_field.setRobotPose(getPose());
    // Logging Output
    log("Odometry/Rotation3d", m_navx2.getRotation3d());

    log("SwerveStates/Target Chassis Speeds", m_chassisSpeeds);

    log("Odometry/Robot", getPose());
  }
}
