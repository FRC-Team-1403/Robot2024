package team1403.robot.swerve;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.opencv.features2d.FlannBasedMatcher;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.NavxAhrs;
import team1403.lib.util.SwerveDriveOdometry;
import team1403.lib.util.SwerveDrivePoseEstimator;
import team1403.robot.Constants;
import team1403.robot.Constants.CanBus;
import team1403.robot.Constants.Swerve;

/**
* The drivetrain of the robot. Consists of for swerve modules and the
* gyroscope.
*/
public class SwerveSubsystem extends SubsystemBase  {
  private final NavxAhrs m_navx2;
  private final SwerveModule[] m_modules;
  private int tagCount = 0;

 private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
 private SwerveModuleState[] m_states = new SwerveModuleState[4];
 private final SwerveDrivePoseEstimator m_odometer;
 private Field2d m_field = new Field2d();


 private Translation2d frontRight = new Translation2d(
     Constants.Swerve.kTrackWidth / 2.0,
     -Constants.Swerve.kWheelBase / 2.0);

 private Translation2d frontLeft = new Translation2d(
     Constants.Swerve.kTrackWidth / 2.0,
     Constants.Swerve.kWheelBase / 2.0);

 private Translation2d backRight = new Translation2d(
     -Constants.Swerve.kTrackWidth / 2.0,
    -Constants.Swerve.kWheelBase / 2.0);

 private Translation2d backLeft = new Translation2d(
     -Constants.Swerve.kTrackWidth / 2.0,
     Constants.Swerve.kWheelBase / 2.0);

 private final PIDController m_driftCorrectionPid = new PIDController(0.25, 0, 0);
 private double m_desiredHeading = 0;
//  private double m_speedLimiter = 0.6;

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
  //  super("Swerve Subsystem", parameters);
   m_navx2 = new NavxAhrs("Gyroscope", SerialPort.Port.kMXP);
   m_Limelight = limelight;
   m_modules = new SwerveModule[] {
       new SwerveModule("Front Left Module",
           CanBus.frontLeftDriveId, CanBus.frontLeftSteerId,
           CanBus.frontLeftEncoderId, Swerve.frontLeftEncoderOffset),
       new SwerveModule("Front Right Module",
           CanBus.frontRightDriveId, CanBus.frontRightSteerId,
           CanBus.frontRightEncoderId, Swerve.frontRightEncoderOffset),
       new SwerveModule("Back Left Module",
           CanBus.backLeftDriveId, CanBus.backLeftSteerId,
           CanBus.backLeftEncoderId, Swerve.backLeftEncoderOffset),
       new SwerveModule("Back Right Module",
           CanBus.backRightDriveId, CanBus.backRightSteerId,
           CanBus.backRightEncoderId, Swerve.backRightEncoderOffset),
   };
  AutoBuilder.configureHolonomic(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              this::driveNoOffset, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
              new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                      new PIDConstants(Swerve.kPTranslation, Swerve.kITranslation, Swerve.kDTranslation), // Translation PID constants
                      new PIDConstants(Swerve.kPAutoTurning, Swerve.kIAutoTurning, Swerve.kDAutoTurning), // Rotation PID constants
                      Constants.Swerve.kMaxSpeed, // Max module speed, in m/s
                      Math.hypot(Swerve.kTrackWidth/2.0, Swerve.kWheelBase/2.0), // Drive base radius in meters. Distance from robot center to furthest module.
                      new ReplanningConfig() // Default path replanning config. See the API for the options here
              ),
              () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  // Optional<Alliance> alliance = DriverStation.getAlliance();
                  // if (alliance.isPresent()) {
                  //   return alliance.get() == DriverStation.Alliance.Red;
                  // }
                  return false;
              },
              this // Reference to this subsystem to set requirements
      );
  m_odometer = new SwerveDrivePoseEstimator(Swerve.kDriveKinematics, new Rotation2d(),
       getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
  m_odometer.update(getGyroscopeRotation(), getModulePositions());


  //  addDevice(m_navx2.getName(), m_navx2);
   if(m_navx2.isConnected())
    while (m_navx2.isCalibrating());

   m_desiredHeading = getGyroscopeRotation().getDegrees();

   setRobotRampRate(0.0);
   setRobotIdleMode(IdleMode.kBrake);

   m_offset = new Translation2d();
   m_rollOffset = -m_navx2.getRoll();
 }

 public void setDisableVision(boolean disable)
 {
  m_disableVision = disable;
 }

 /**
  * Gets the 4 swerve module positions.
  *
  * @return an array of swerve module positions
  */
 public SwerveModulePosition[] getModulePositions() {
   return new SwerveModulePosition[] {
       m_modules[0].getPosition(),
       m_modules[1].getPosition(),
       m_modules[2].getPosition(),
       m_modules[3].getPosition()
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
 public Pose2d getPose() {
   return m_odometer.getEstimatedPosition();
 }

 /**
  * Set the position o
  f thte odometry.
  *
  * @param pose the new position of the odometry.
  */
 public void setPose(Pose2d pose) {
   m_odometer.setPose(pose);
 }

 public SwerveDrivePoseEstimator getOdometer() {
   return m_odometer;
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
     m_modules[i].set((states[i].speedMetersPerSecond
         / Swerve.kMaxSpeed),
         states[i].angle.getRadians());
   }
 }

 public ChassisSpeeds getChassisSpeed() {
   return m_chassisSpeeds;
 }

 /**
  * Sets which wheel to pivot for the robot.
  * 
  * @param isRight whether right or left wheels should be pivoted
  */
 public void setPivotAroundOneWheel(boolean isRight) {
   // Right wheels are to be pivoted
   if (isRight) {
     if ((45.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < -45.0)
         || (315.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < -315.0)) {
       if (m_chassisSpeeds.vxMetersPerSecond >= 0.0) {
         // Pivots around front right wheel
         m_offset = frontRight;
       } else {
         // Pivot around back right wheel
         m_offset = backRight;
       }
     } else if ((-135.0 < getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() <= -45.0)
         || (225.0 < getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() <= 315.0)) {
       if (m_chassisSpeeds.vxMetersPerSecond >= 0.0) {
         // Pivot around back right wheel
         m_offset = backRight;
       } else {
         // Pivot around back left wheel
         m_offset = backLeft;
       }
     } else if ((-225.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < -135.0)
         || (135.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < 225.0)) {
       if (m_chassisSpeeds.vxMetersPerSecond >= 0.0) {
         // Pivot around back left wheel
         m_offset = backLeft;
       } else {
         // Pivot around front left wheel
         m_offset = frontLeft;
       }
     } else if ((-315.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < -225.0)
         || (45.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < 135.0)) {
       if (m_chassisSpeeds.vxMetersPerSecond >= 0.0) {
         // Pivot around front left wheel
         m_offset = frontLeft;
       } else {
         // Pivot around front right wheel
         m_offset = frontRight;
       }
     }
     // Left wheels are to be pivoted
   } else {
     if ((45.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < -45.0)
         || (315.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < -315.0)) {
       if (m_chassisSpeeds.vxMetersPerSecond >= 0.0) {
         // Pivots around front left wheel
         m_offset = frontLeft;
       } else {
         // Pivot around back left wheel
         m_offset = backLeft;
       }
     } else if ((-135.0 < getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() <= -45.0)
         || (225.0 < getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() <= 315.0)) {
       if (m_chassisSpeeds.vxMetersPerSecond >= 0.0) {
         // Pivot around back left wheel
         m_offset = backLeft;
       } else {
         // Pivot around back right wheel
         m_offset = backRight;
       }
     } else if ((-225.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < -135.0)
         || (135.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < 225.0)) {
       if (m_chassisSpeeds.vxMetersPerSecond >= 0.0) {
         // Pivot around back right wheel
         m_offset = backRight;
       } else {
         // Pivot around front right wheel
         m_offset = frontRight;
       }
     } else if ((-315.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < -225.0)
         || (45.0 <= getGyroscopeRotation().getDegrees()) && (getGyroscopeRotation().getDegrees() < 135.0)) {
       if (m_chassisSpeeds.vxMetersPerSecond >= 0.0) {
         // Pivot around front right wheel
         m_offset = frontRight;
       } else {
         // Pivot around front left wheel
         m_offset = frontLeft;
       }
     }
   }
 }

 /**
  * Resets the robot to no longer pivot around one wheel.
  */
 public void setMiddlePivot() {
   m_offset = new Translation2d();
 }

 /**
  * Puts the drivetrain into xMode where all the wheel put towards the center of the robot,
  * making it harder for the robot to be pushed around. 
  */
 private void xMode() {
   SwerveModuleState[] states = {
       // Front Left
       new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
       // Front Right
       new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
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
  * Adds rotational velocity to the chassis speed to compensate for
  * unwanted changes in gyroscope heading.
  * 
  * @param chassisSpeeds the given chassisspeeds
  * @return the corrected chassisspeeds
  */
 private ChassisSpeeds translationalDriftCorrection(ChassisSpeeds chassisSpeeds) {
    if(!m_navx2.isConnected())
      return chassisSpeeds;
   double translationalVelocity = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
   SmartDashboard.putNumber("translationVelocity", translationalVelocity);
   SmartDashboard.putNumber("Desired Heading", m_desiredHeading);
   SmartDashboard.putNumber("Anuglar vel", m_navx2.getAngularVelocity());
   if (Math.abs(m_navx2.getAngularVelocity()) > 0.5) {
     m_desiredHeading = getGyroscopeRotation().getDegrees();
   } else if (translationalVelocity > 0.2 && Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= 0.1) {
     double calc = m_driftCorrectionPid.calculate(getGyroscopeRotation().getDegrees(), m_desiredHeading);
     if (Math.abs(calc) >= 0.35) {
       chassisSpeeds.omegaRadiansPerSecond += calc;
     }
   }
   return chassisSpeeds;
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

 public NavxAhrs getNavxAhrs(){
  return m_navx2;
 }

 private ChassisSpeeds rotationalDriftCorrection(ChassisSpeeds chassisSpeeds) {
   // Assuming the control loop runs in 20ms
   final double deltaTime = 0.02;

   if(!m_navx2.isConnected())
   {
    return chassisSpeeds;
   }

   // The position of the bot one control loop in the future given the chassisspeed
   Pose2d robotPoseVel = new Pose2d(chassisSpeeds.vxMetersPerSecond * deltaTime,
       chassisSpeeds.vyMetersPerSecond * deltaTime,
       new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * deltaTime));

   Twist2d twistVel = new Pose2d(0, 0, new Rotation2d()).log(robotPoseVel);
   return new ChassisSpeeds(
       twistVel.dx / deltaTime, twistVel.dy / deltaTime,
       twistVel.dtheta / deltaTime);
 }

 @Override
 public void periodic() {
  SmartDashboard.putNumber("Gyro Reading", getGyroscopeRotation().getDegrees());

  if (m_Limelight.hasTarget()) {
    Pose2d pose = m_Limelight.getDistance2D();
    var x = m_Limelight.getPosStdv();
    if(pose != null && !m_disableVision && tagCount > 0)
    {
      // m_navx2.setAngleOffset(pose.getRotation().getDegrees());
      // m_desiredHeading = pose.getRotation().getDegrees();
      m_odometer.addVisionMeasurement(pose, Timer.getFPGATimestamp(), x);
    }
    else if(pose != null && !m_disableVision && tagCount == 0)
    {
      m_odometer.setPose(pose);
      // // m_navx2.setAngleOffset(pose.getRotation().getDegrees());
      // m_desiredHeading = pose.getRotation().getDegrees();
      tagCount++;
    }
  } else {
    m_odometer.update(getGyroscopeRotation(),getModulePositions());
  }

   SmartDashboard.putString("Odometry", m_odometer.getEstimatedPosition().toString());
   //SmartDashboard.putNumber("Speed", m_speedLimiter);
   SmartDashboard.putNumber("Roll Degrees", getGyroscopeRotation().getDegrees());

   if (this.m_isXModeEnabled) {
     xMode();
   } else {
      m_chassisSpeeds = translationalDriftCorrection(m_chassisSpeeds);
     //m_chassisSpeeds = rotationalDriftCorrection(m_chassisSpeeds);

     m_states = Swerve.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds, m_offset);
     setModuleStates(m_states);
   }

   SmartDashboard.putNumber("Front Left Absolute Encoder", m_modules[0].getAbsoluteAngle());
   SmartDashboard.putNumber("Front Right Absolute Encoder", m_modules[1].getAbsoluteAngle());
   SmartDashboard.putNumber("Back Left Absolute Encoder", m_modules[2].getAbsoluteAngle());
   SmartDashboard.putNumber("Back Right Absolute Encoder", m_modules[3].getAbsoluteAngle());
   m_field.setRobotPose(m_odometer.getEstimatedPosition());
   // Logging Output

   Logger.recordOutput("Odometer", m_odometer.getEstimatedPosition().toString());
  //  Logger.recordOutput("Speed", m_speedLimiter);
   Logger.recordOutput("Amount of Roll", getGyroRoll());

   Logger.recordOutput("Chassis Speeds", m_chassisSpeeds.toString());
   Logger.recordOutput("Front Left Absolute Encoder Angle", m_modules[0].getAbsoluteAngle());
   Logger.recordOutput("Front Right Absolute Encoder Angle", m_modules[1].getAbsoluteAngle());
   Logger.recordOutput("Back Left Absolute Encoder Angle", m_modules[2].getAbsoluteAngle());
   Logger.recordOutput("Back Right Absolute Encoder Angle", m_modules[3].getAbsoluteAngle());


   Logger.recordOutput("Gyro Reading", getGyroscopeRotation().getDegrees());


 }
}
