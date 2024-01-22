package team1403.robot;

import java.util.Arrays;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import team1403.lib.util.Dimension;


/**
 * This class holds attributes for the robot configuration.
 *
 * <p>
 * The RobotConfig is broken out into different areas,
 * each of which is captured in a class for that area. Each
 * subsystem has its own independent config.
 *
 * <p>
 * The "electrical" configs are treated separate and independent
 * to make it easier to see how the robot should be wired and see
 * any conflicts since these ports specify their config together.
 */
public class Constants {

  //Variables to used by all subsystems.
  public static final Dimension robotDimensions = new Dimension(0, 0, 0);
  public static final Dimension wristDimensions = new Dimension(0, 0, 0); //TODO


  public static double kRobotHeight = 32;
  public static double kHeightFromGround = 33.465;
  public static double kGroundToTopOfFrame = 1.465;
  public static double kFrameHeight = 2;

  /**
   * Swerve Constants.
   * 
   */
  public static class Swerve {
    public static final int kEncoderResetIterations = 500;
    public static final double kEncoderResetMaxAngularVelocity = Math.toRadians(0.5);
    public static final int kStatusFrameGeneralPeriodMs = 250;
    public static final int kCanTimeoutMs = 250;

    public static final double kPTurning = 0.3;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0;

    public static final double kPAutoTurning = 3;
    public static final double kIAutoTurning = 0.0;
    public static final double kDAutoTurning = 0.0;

    public static final double kPTranslation = 12;
    public static final double kITranslation = 0.0;
    public static final double kDTranslation = 0.5;

    public static final double kTrackWidth = Units.inchesToMeters(25.6);
    public static final double kWheelBase = Units.inchesToMeters(26.75);

    public static final SwerveDriveKinematics kDriveKinematics = 
        new SwerveDriveKinematics(
            // Front left
            new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
            // Front right
            
            new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
            // Back left
            new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
            // Back right
            new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

    public static final double frontLeftEncoderOffset = -(4.667903537536006 - Math.PI);
    public static final double frontRightEncoderOffset = -(4.936350175415994 - Math.PI);
    public static final double backLeftEncoderOffset = -(4.743068596142402 - Math.PI); //4.743068596142402
    public static final double backRightEncoderOffset = -(0.593650564911743 + Math.PI);//

    public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double kSteerReduction = (15.0 / 32.0) * (10.0 / 60.0);

    public static final double kSteerRelativeEncoderPositionConversionFactor = 2.0 * Math.PI
        * Swerve.kSteerReduction;

    public static final double kSteerRelativeEncoderVelocityConversionFactor = 2.0 * Math.PI
        * Swerve.kSteerReduction / 60.0;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kMaxSpeed = 14.5;

    public static final double kMaxAngularSpeed = 50;
    // (kMaxSpeed / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0)); // 39.795095397

    public static final double kVoltageSaturation = 12.0;
    public static final int kCurrentLimit = 40;

    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints
        = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed,
        kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  /**
   * Configures the CAN bus. These are grouped together
   * rather than by subsystem to more easily detect conflict
   * and understand overall wiring.
   */
  public static class CanBus {

    // Swerve CanBus ids
    public static final int frontLeftDriveId = 16;
    public static final int frontLeftSteerId = 5;
    public static final int frontLeftEncoderId = 3;

    public static final int frontRightDriveId = 9;
    public static final int frontRightSteerId = 12;
    public static final int frontRightEncoderId = 4;

    public static final int backLeftDriveId = 8;
    public static final int backLeftSteerId = 7;
    public static final int backLeftEncoderId = 2;

    public static final int backRightDriveId = 14;
    public static final int backRightSteerId = 6;
    public static final int backRightEncoderId = 1;
    public static final int intakeAndShooterMotorTop = 0;
    public static final int intakeAndShooterMotorBottom = 0;
    public static final int hangerMotor = 0;    
  }

  public static class Turret {
    public static final double absEncoderGearRatio = 18.0/120.0;
    public static final int absEncoderPort = 5;
    public static final int hallEffectPort = 1;
    // checked with phoenix
    public static final int turretMotor = 19;
  
  }

  public static class Vision {
    public static final double rotationCutoff = 0.2;
  }

  /**
   * Ports on the RoboRIO.
   */
  public static class RioPorts {

    public static final int intakeLimitSwitchPort = 0;
    public static final int LEDPort = 0;
  }

  /**
   * Config parameters for tuning the operator interface.
   */
  public static class Operator {

    public static final int dPadUp = 0;
    public static final int dPadRight = 1;
    public static final int dPadDown = 2;
    public static final int dPadLeft = 3;

    /**
     * The joystick port for the operator's controller.
     */
    public static final int pilotPort = 0;

    /**
     * Encoder ticks from center still considered close enough to be at center.
     */
    public static final double seekCenterTolerance = 10.0;
  }

  /**
   * Config parameters for tuning the operator interface.
   */
  public static class Driver {

    /**
     * The joystick port for the operator's controller.
     */
    public static final int pilotPort = 1;

    /**
     * Encoder ticks from center still considered close enough to be at center.
     */
    public static final double seekCenterTolerance = 10.0;
  }

  public static class Hanger {

public static int channel;

  }

}