package team1403.robot;

import java.util.Arrays;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import team1403.lib.util.Dimension;
import team1403.robot.subsystems.arm.ArmState;

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

  // Variables to used by all subsystems.
  public static final Dimension robotDimensions = new Dimension(0, 0, 0);
  public static final Dimension wristDimensions = new Dimension(0, 0, 0); // TODO

  public static double kRobotHeight = 32;
  public static double kHeightFromGround = 33.465;
  public static double kGroundToTopOfFrame = 1.465;
  public static double kFrameHeight = 2;

  /**
   * Swerve Constants.
   * 
   */
  public static class Swerve {
    // around 1 second
    public static final int kEncoderResetIterations = 50;
    public static final double kEncoderResetMaxAngularVelocity = Math.toRadians(0.5);
    public static final int kStatusFrameGeneralPeriodMs = 250;
    public static final int kCanTimeoutMs = 250;

    public static final double kPTurning = 0.4;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0;

    public static final double kPAutoTurning = 3;
    public static final double kIAutoTurning = 0.0;
    public static final double kDAutoTurning = 0.0;

    public static final double kPTranslation = 12;
    public static final double kITranslation = 0.0;
    public static final double kDTranslation = 0.5;

    public static final double kTrackWidth = Units.inchesToMeters(23);
    public static final double kWheelBase = Units.inchesToMeters(24);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
        // Front right
     
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
        // Back left  
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
        // Back right
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

    public static final double frontLeftEncoderOffset = -Math.PI;
    public static final double frontRightEncoderOffset = -0.55;
    public static final double backLeftEncoderOffset = 0; //4.743068596142402
    public static final double backRightEncoderOffset = 1.25;//-0.2966

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kDrivePositionConversionFactor = 0.05215454470665408; //(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * kWheelDiameterMeters * Math.PI; //0.05215454470665408
    public static final double kSteerReduction = (15.0 / 32.0) * (10.0 / 60.0);

    public static final double kSteerRelativeEncoderPositionConversionFactor = 2.0 * Math.PI
        * Swerve.kSteerReduction;

    public static final double kSteerRelativeEncoderVelocityConversionFactor = 2.0 * Math.PI
        * Swerve.kSteerReduction / 60.0;

    public static final double kMaxSpeed = 6;

    public static final double kMaxAngularSpeed = (kMaxSpeed / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0)); // 39.795095397


    public static final double kVoltageSaturation = 12.0;
    public static final int kCurrentLimit = 40;

    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed,
        kMaxAngularAccelerationRadiansPerSecondSquared);

    //front-to-back-disp = ~8.568 inches 
    //left-to-right-disp = 0 inches
    //top-to-bottom disp = 17.82426 inches
    public static final Transform3d kCameraOffset = new Transform3d(0.0,0.0,Units.inchesToMeters(17.82426), new Rotation3d(0.0,Units.degreesToRadians(25), Math.PI));
  }

  /**
   * Configures the CAN bus. These are grouped together
   * rather than by subsystem to more easily detect conflict
   * and understand overall wiring.
   */
  public static class CanBus {

    private static final int kTBD = 0;

    // Swerve CanBus ids
    public static final int frontLeftDriveID = 13;
    public static final int frontLeftSteerID = 12;
    public static final int frontLeftEncoderID = 22;

    public static final int frontRightDriveID = 9;
    public static final int frontRightSteerID = 8;
    public static final int frontRightEncoderID = 20;

    public static final int backLeftDriveID = 11;
    public static final int backLeftSteerID = 10;
    public static final int backLeftEncoderID = 21;

    public static final int backRightDriveID = 7;
    public static final int backRightSteerID = 6;
    public static final int backRightEncoderID = 23;

    // pivot motor ports (arm)
    public static final int rightPivotMotorID = 5;
    public static final int leftPivotMotorID = 14;

    // hanger ID
    public static final int rightHangerMotorID = 27;
    public static final int leftHangerMotorID = 28;

    // intake and shooter IDs
    public static final int shooterMotorTopID = 2;
    public static final int shooterMotorBottomID = 1;
    public static final int intakeMotorID = 4;

    // wrist
    public static final int wristMotorID = 15;

    // other
    public static final int powerDistributionID = 60;
  }
  
  /**
   * Ports on the RoboRIO.
   */
  public static class RioPorts {
    // actual
    public static final int LEDPort = 0;
    public static final int intakePhotogate1 = 3;
    public static final int shooterPhotogate = 2;
    public static final int kArmAbsoluteEncoder = 0;
    //Wrist 
    public static final int kwristAbsoluteEncoder = 1; // DIO
    //Hanger
    public static final int kleftServoID = 8;
    public static final int krightServoID = 9;
    public static final int kHangerLimitRightBottomID = 0;
    public static final int kHangerLimitLeftBottomID = 0;
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
  
  public static class Vision {
    public static final double rotationCutoff = 5;
    public static boolean isRotated = false;
    public static final AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(Arrays.asList(
        new AprilTag(1, (new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)))),
        new AprilTag(2, new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI))),
        new AprilTag(3, new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI))),
        new AprilTag(4, new Pose3d(
            Units.inchesToMeters(636.96),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d(0.0, 0.0, Math.PI))),
        new AprilTag(5, new Pose3d(
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d())),
        new AprilTag(6, new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI))),
        new AprilTag(7, new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI))),
        new AprilTag(8, new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d()))),
        Units.inchesToMeters(651.25), Units.inchesToMeters(315.5));
  }

  public static class Arm {
    // all angles are in degrees
    public static final double KPArmPivot = 0.0125;
    public static final double KIArmPivot = 0.0;
    public static final double KDArmPivot = 0;
    public static final double kAbsolutePivotOffset = 0;
    public static final double kFeedforwardG = 0.03;
    public static final double kFeedforwardV = 0.0001;

    public static final double kMaxPivotAngle = 230;//180
    public static final double kMinPivotAngle = 75;
    public static final double kPivotMotorMaxAmperage = 40;

    public static final int kPivotMotorCurrentLimit = 25;
    public static final double kPivotMotorVoltageLimit = 12;
    public static final ArmState KStageLineSetPoint = null;

    public static  double kIntakeSetpoint = 90;
    public static  double kAmpSetpoint = 210;
    public static  double kLoadingSetpoint = 150;
    public static  double kDriveSetpoint = 114;
    public static  double kDefaultClose = 114;
  }



  public static class IntakeAndShooter {
    public static final double kFrameAngle = 250.24629;
    public static final double kFrameClearanceAngle = 234.5; // cone angle
    public static final double kHorizonAngle = 210; 
    public static final double kSpeedReduction = 2.0; // test value
    public static double kStageLineRPM = 5000; //To test
    public static double kCenterLineRPM = 6000;
    public static double kLaunchpadRPM = 5000;
    public static final double kCloseRPM = 4800;
    public static final double kExpelDeadzone = 0.15;
  }

  public static class Auto {
    public static boolean kAmp = false;
    public static boolean kCenterLine = false;
    public static boolean kStageLine = false;
    public static boolean kDriveSetpoint = false;
    public static boolean kFinished = false;
    public static boolean kInAuto = false;
    public static boolean kisIntaked = false;
    public static boolean kSide = false;
  }

  public static class Hanger {
    public static final double kTopLimit = 36;
    public static final double kBottomLimit = 3;
    public static final double kLockAngle = 170;
    public static final double kUnlockAngle = 180;
  }
  
  public static class Wrist {
    public static final double kWristConversionFactor = 0;
    public static final double kAbsoluteWristOffset = 0;

    public static double KPWrist = 0.0097; //original value 0.0092 changed - 0.0097
    public static final double KIWrist = 0.0;
    public static final double KDWrist = 0;

    public static final double kTopLimit = 180;
    public static final double kBottomLimit = 0;

    public static  double kIntakeSetpoint = 134;
    public static  double kAmpSetpoint = 160.5;
    public static double kAmpShoootingSetpoint = 142;
    public static  double kLoadingSetpoint = 90;
    public static  double kDriveSetpoint = 140;//140
    public static  double kDefaultClose = 136;
    public static double kStageLineSetpoint = 138;//To test
    public static double kStageLineSideSetpoint = 135;//136 version 2
    public static double kSideLineSourceSetpoint = 130; 
    public static double kLaunchpadSetpoint = 140;
    public static double kCenterLineSetpoint = 133;//115
    public static  double kShootingAngle = 147;//147 for teleop working


    public static final double kWristUpperLimit = 150;
    public static final double kWristLowerLimit = 130;
    public static final double kWristConstraint = 140;
    public static final double kArmConstraint = 120;
  }
}