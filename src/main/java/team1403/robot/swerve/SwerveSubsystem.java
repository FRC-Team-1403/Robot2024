package team1403.robot.swerve;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private int tagCount = 0;
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private SwerveModuleState[] m_states = new SwerveModuleState[4];
  private final SwerveDrivePoseEstimator m_odometer;
  private Field2d m_field = new Field2d();

  private final PIDController m_driftCorrectionPid = new PIDController(0.05, 0, 0);
  private double m_desiredHeading = 0;
  // private double m_speedLimiter = 0.6;

  private Translation2d m_offset;

  private double m_rollOffset;

  private boolean m_isXModeEnabled = false;
  private Limelight m_Limelight;
  private boolean m_disableVision = false;

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
  public SwerveSubsystem(Limelight limelight) {
    SmartDashboard.putData("Field", m_field);
    // super("Swerve Subsystem", parameters);
    m_navx2 = new NavxAhrs("Gyroscope", SerialPort.Port.kMXP);
    m_Limelight = limelight;
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
        () -> Swerve.kDriveKinematics.toChassisSpeeds(getModuleStates()), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveNoOffset, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(Swerve.kPTranslation, Swerve.kITranslation, Swerve.kDTranslation), // Translation PID
                                                                                                // constants
            new PIDConstants(Swerve.kPAutoTurning, Swerve.kIAutoTurning, Swerve.kDAutoTurning), // Rotation PID
                                                                                                // constants
            Constants.Swerve.kMaxSpeed, // Max module speed, in m/s
            Math.hypot(Swerve.kTrackWidth / 2.0, Swerve.kWheelBase / 2.0), // Drive base radius in meters. Distance from
                                                                           // robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),

        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          Optional<Alliance> alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return true;
        },
        this // Reference to this subsystem to set requirements
    );

    // addDevice(m_navx2.getName(), m_navx2);
    if (m_navx2.isConnected())
      while (m_navx2.isCalibrating());

    zeroGyroscope();

    m_odometer = new SwerveDrivePoseEstimator(Swerve.kDriveKinematics, new Rotation2d(),
        getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
    m_odometer.update(getGyroscopeRotation(), getModulePositions());

    m_driftCorrectionPid.enableContinuousInput(-180, 180);

    m_desiredHeading = getGyroscopeRotation().getDegrees();

    setRobotRampRate(0.0);
    setRobotIdleMode(IdleMode.kBrake);

    m_offset = new Translation2d();
    m_rollOffset = -m_navx2.getRoll();
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
    return new SwerveModulePosition[] {
        m_modules[0].getModulePosition(),
        m_modules[1].getModulePosition(),
        m_modules[2].getModulePosition(),
        m_modules[3].getModulePosition()
    };
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

  public SwerveDrivePoseEstimator getOdometer() {
    return m_odometer;
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
  public void zeroGyroscope() {
    // tracef("zeroGyroscope %f", getGyroscopeRotation());
    m_navx2.reset();
    m_desiredHeading = 0;
  }

  /**
   * Return the position of the drivetrain.
   *
   * @return the position of the drivetrain in Pose2d
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_odometer.getEstimatedPosition();
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
    m_odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
  }

  /**
   * Gets the heading of the gyroscope.
   *
   * @return a Rotation2d object that contains the gyroscope's heading
   */
  public Rotation2d getGyroscopeRotation() {
    return m_navx2.get180to180Rotation2d();
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
    m_chassisSpeeds = chassisSpeeds;
    m_offset = offset;
    SmartDashboard.putString("Chassis Speeds", m_chassisSpeeds.toString());
  }

  public void driveNoOffset(ChassisSpeeds chassisSpeeds) {
    drive(chassisSpeeds, new Translation2d());
    SmartDashboard.putString("Chassis Speeds", m_chassisSpeeds.toString());
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
      Rotation2d rot = new Rotation2d(m_modules[i].getAbsoluteAngle());
      states[i] = SwerveModuleState.optimize(states[i], rot);
      m_modules[i].set(states[i]);
    }
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
      m_modules[0].getState(),
      m_modules[1].getState(),
      m_modules[2].getState(),
      m_modules[3].getState()
    };
    return states;
  }

  public ChassisSpeeds getChassisSpeed() {
    return m_chassisSpeeds;
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
    SwerveModuleState[] states = {
        // Front Left
        new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
        // Front Right
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        // Back left
        new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
        // Back Right
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };
    setModuleStates(states);
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

  public NavxAhrs getNavxAhrs() {
    return m_navx2;
  }

  private double prevTimeStep = 0;

  private ChassisSpeeds translationalDriftCorrection(ChassisSpeeds chassisSpeeds) {
    double curTimeStep = Timer.getFPGATimestamp();
    if(prevTimeStep == 0)
    {
      prevTimeStep = curTimeStep;
      return chassisSpeeds;
    }
    
    final double deltaTime = curTimeStep - prevTimeStep;
    prevTimeStep = curTimeStep;
    
    return ChassisSpeeds.discretize(chassisSpeeds, deltaTime);
  }

  private ChassisSpeeds rotationalDriftCorrection(ChassisSpeeds speeds) {

    //implement another algorithm for this later

    return speeds;
  }

  private void updateSwerveModules() {
    for(SwerveModule module : m_modules) {
      module.periodic();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Reading", getGyroscopeRotation().getDegrees());

    updateSwerveModules();

    if (m_Limelight.hasTarget() && !m_disableVision) {
      Pose2d pose = m_Limelight.getDistance2D();
      if (pose != null) {
        m_odometer.addVisionMeasurement(pose, Timer.getFPGATimestamp());
      }
      else {
        m_odometer.update(getGyroscopeRotation(), getModulePositions());
      }
    } else {
      m_odometer.update(getGyroscopeRotation(), getModulePositions());
    }

    SmartDashboard.putString("Odometry", getPose().toString());
    // SmartDashboard.putNumber("Speed", m_speedLimiter);

    if (this.m_isXModeEnabled) {
      xMode();
    } else {
      m_chassisSpeeds = translationalDriftCorrection(m_chassisSpeeds);
      if (DriverStation.isTeleop()) m_chassisSpeeds = rotationalDriftCorrection(m_chassisSpeeds);

      m_states = Swerve.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds, m_offset);
      setModuleStates(m_states);
    }
    m_field.setRobotPose(getPose());
    // Logging Output
    Logger.recordOutput("Gyro Roll", getGyroRoll());

    Logger.recordOutput("Chassis Speeds", Swerve.kDriveKinematics.toChassisSpeeds(getModuleStates()).toString());

    // Logger.recordOutput("Front Left Absolute Encoder Angle", m_modules[0].getAbsoluteAngle());
    // Logger.recordOutput("Front Right Absolute Encoder Angle", m_modules[1].getAbsoluteAngle());
    // Logger.recordOutput("Back Left Absolute Encoder Angle", m_modules[2].getAbsoluteAngle());
    // Logger.recordOutput("Back Right Absolute Encoder Angle", m_modules[3].getAbsoluteAngle());

    Logger.recordOutput("Gyro Reading", getGyroscopeRotation().getDegrees());
    Logger.recordOutput("Desired Heading", m_desiredHeading);
  }
}
