package team1403.robot.swerve;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.simulation.SwerveModuleSimulation;
import swervelib.telemetry.SwerveDriveTelemetry;
import team1403.lib.device.wpi.NavxAhrs;
import team1403.robot.Constants;
import team1403.robot.Constants.CanBus;
import team1403.robot.Constants.Swerve;

/**
 * The drivetrain of the robot. Consists of for swerve modules and the
 * gyroscope.
 */
public class SwerveSubsystem extends SubsystemBase {
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();

  private boolean m_isXModeEnabled = false;
  private Limelight m_Limelight;
  private boolean m_disableVision = false;

  private SwerveDrive m_swerve;

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
    // super("Swerve Subsystem", parameters);
    m_Limelight = limelight;

    try {
      m_swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo")).createSwerveDrive(Constants.Swerve.kMaxSpeed);
    } catch (IOException e) {
      e.printStackTrace();
    }
    
    //although we use choreo, path planner is still useful for things like pathfinding
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        () -> m_swerve.getRobotVelocity(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
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
    Pathfinding.setPathfinder(new LocalADStar());
    PathPlannerLogging.setLogActivePathCallback((activePath) -> {
      Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
    });
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
        Logger.recordOutput("Odometery/TrajectorySetpoint", targetPose);
    });

    zeroGyroscope();
    m_swerve.setMotorIdleMode(true);
    //update very often to be accurate
    m_swerve.setOdometryPeriod(0.005);
    m_swerve.setHeadingCorrection(false);
    m_swerve.setCosineCompensator(false);
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
    return m_swerve.getModulePositions();
  }
  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' directi=on.
   */
  public void zeroGyroscope() {
    // tracef("zeroGyroscope %f", getGyroscopeRotation());
    m_swerve.zeroGyro();
  }

  /**
   * Return the position of the drivetrain.
   *
   * @return the position of the drivetrain in Pose2d
   */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_swerve.getPose();
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
    m_swerve.resetOdometry(pose);
  }

  /**
   * Gets the heading of the gyroscope.
   *
   * @return a Rotation2d object that contains the gyroscope's heading
   */
  public Rotation2d getGyroscopeRotation() {
    return m_swerve.getOdometryHeading();
  }

  /**
   * Gets the roll of the gyro (Y axis of gyro rotation).
   * 
   * @return a double representing the roll of robot in degrees
   */
  public double getGyroRoll() {
    return m_swerve.getRoll().getDegrees();
  }

  /**
   * Gets the pitch of the gyro (X axis of gyro rotation).
   * 
   * @return a double representing the pitch of robot in degrees
   */
  public double getGyroPitch() {
    return m_swerve.getPitch().getDegrees();
  }

  /**
   * Moves the drivetrain at the given chassis speeds.
   *
   * @param chassisSpeeds the speed to move at
   * @param offset        the swerve module to pivot around
   */
  public void drive(ChassisSpeeds chassisSpeeds, Translation2d offset) {
    m_chassisSpeeds = chassisSpeeds;
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
    m_swerve.setModuleStates(states, false);

    Logger.recordOutput("SwerveStates/Target", states);
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    return m_swerve.getStates();
  }

  public ChassisSpeeds getChassisSpeed() {
    return m_chassisSpeeds;
  }

  /**
   * Puts the drivetrain into xMode where all the wheel put towards the center of
   * the robot, 
   * making it harder for the robot to be pushed around.
   */
  private void xMode() {
    SwerveModuleState[] states = {
        // Front Left
        new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
        // Front Right
        new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
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

  @Override
  public void periodic() {

    if (m_Limelight.hasTarget() && !m_disableVision) {
      Pose2d pose = m_Limelight.getDistance2D();
      if (pose != null) {
        Logger.recordOutput("Odometery/Vision Measurement", pose);
        m_swerve.addVisionMeasurement(pose, Timer.getFPGATimestamp());
      }
    }

    SwerveDriveTelemetry.updateData();
    // SmartDashboard.putNumber("Speed", m_speedLimiter);

    if (this.m_isXModeEnabled) {
      xMode();
    } else {
      m_swerve.drive(m_chassisSpeeds);
      //estimate of target
      Logger.recordOutput("SwerveStates/Target", m_swerve.kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(m_chassisSpeeds, 0.02)));
    }
    SmartDashboard.putString("Module States", getModuleStates().toString());
    // Logging Output
    Logger.recordOutput("Gyro Roll", getGyroRoll());

    // Logger.recordOutput("Front Left Absolute Encoder Angle", m_modules[0].getAbsoluteAngle());
    // Logger.recordOutput("Front Right Absolute Encoder Angle", m_modules[1].getAbsoluteAngle());
    // Logger.recordOutput("Back Left Absolute Encoder Angle", m_modules[2].getAbsoluteAngle());
    // Logger.recordOutput("Back Right Absolute Encoder Angle", m_modules[3].getAbsoluteAngle());

    Logger.recordOutput("Gyro Reading", getGyroscopeRotation().getDegrees());
  }
}
