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
    public static final int kEncoderResetIterations = 500;
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

    public static final double kTrackWidth = Units.inchesToMeters(17.5);
    public static final double kWheelBase = Units.inchesToMeters(17.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
        // Front right

        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
        // Back left
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
        // Back right
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

    public static final double frontLeftEncoderOffset = -(4.667903537536006 - Math.PI); // 1.70498046875
    public static final double frontRightEncoderOffset = -(0.009203884727314 - Math.PI); // 2.70498046875
    public static final double backLeftEncoderOffset = -(4.680175383839091 - Math.PI); // 1.70498046875
    public static final double backRightEncoderOffset = -(5.776971647177325 + Math.PI);// 2.70498046875

    public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double kSteerReduction = (15.0 / 32.0) * (10.0 / 60.0);

    public static final double kSteerRelativeEncoderPositionConversionFactor = 2.0 * Math.PI
        * Swerve.kSteerReduction;

    public static final double kSteerRelativeEncoderVelocityConversionFactor = 2.0 * Math.PI
        * Swerve.kSteerReduction / 60.0;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kMaxSpeed = 4.5;

    public static final double kMaxAngularSpeed = (kMaxSpeed / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0)); // 39.795095397


    public static final double kVoltageSaturation = 12.0;
    public static final int kCurrentLimit = 40;

    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
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
    public static final int frontLeftDriveId = 13;
    public static final int frontLeftSteerId = 12;
    public static final int frontLeftEncoderId = 3;

    public static final int frontRightDriveId = 9;
    public static final int frontRightSteerId = 8;
    public static final int frontRightEncoderId = 1;

    public static final int backLeftDriveId = 11;
    public static final int backLeftSteerId = 10;
    public static final int backLeftEncoderId = 2;

    public static final int backRightDriveId = 7;
    public static final int backRightSteerId = 6;
    public static final int backRightEncoderId = 4;

    public static final int intakeAndShooterMotorTop = 0;
    public static final int intakeAndShooterMotorBottom = 0;
   //intake port
    public static final int intakeMotor = 4;

    //pivot motor ports (shoulder)
    public static final int rightPivotMotorID = 0;
    public static final int leftPivotMotorID = 0;

    // hanger ID
    public static final int rightHangerMotor = 0;
    public static final int leftHangerMotor = 0;

    //intake and shooter IDs
    public static final int shooterMotorTopID = 0;
    public static final int shooterMotorBottomID = 0;
    public static final int intakeMotorID = 0;

    //wrist 
    public static final int wristMotor = 15;
  }

  public static class Turret {
    public static final double absEncoderGearRatio = 18.0 / 120.0;
    public static final int absEncoderPort = 5;
    public static final int hallEffectPort = 1;
    // checked with phoenix
    public static final int turretMotor = 19;
  }

  /**
   * Ports on the RoboRIO.
   */
  public static class RioPorts {
    // remove
    public static final int kWristAbsoluteEncoder = 1; // DIO

    public static final int kExtensionMinMagneticSwitch = 2; // DIO
    public static final int kExtensionMaxMagneticSwitch = 3; // DIO
    // actual
    public static final int intakeLimitSwitchPort = 0;
    public static final int LEDPort = 0;
    public static final int intakePhotogate1 = 0;
    public static final int shooterPhotogate = 0;
    public static final int kArmLimitSwitch = 0;
    public static final int kArmAbsoluteEncoder = 0;
    //photogates
    public static final int shooterPhotoswitch = 0;
    public static final int kIntakePhotogae2 = 0;
    //Wrist 
    public static final int kwristAbsoluteEncoder = 0;
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
    public static final double KPArmPivot = 0;
    public static final double KIArmPivot = 0;
    public static final double KDArmPivot = 0;
    public static int kAbsolutePivotOffset;
    public static double kMaxPivotAngle;
    public static double kMinPivotAngle;
    public static double kPivotAngleMaxAmperage;
    public static double kDecreaseIntakeSpeed;
  }

  public static class IntakeAndShooter {
    public static final double kDecreaseIntakeSpeed = 2; //test value 
    public static final double kAbsolutePivotOffset = 40;
    public static final double kMaxPivotAngle = 100.0;
    public static final double kFrameAngle = 250.24629;
    public static final double kFrameClearanceAngle = 234.5; // cone angle
    public static final double kMinPivotAngle = 132.211;
    public static final double kPivotAngleMaxAmperage = 40;
    public static final double kHorizonAngle = 210;
    public static final double kPivotLimitSwitchOffset = -6;
    public static final double kBaseArmLength = 0;

    public static final double kArmConversionFactor = 1;

    public static final int m_shooterSpeed = 0;
  }

  public static class Wrist {
    public static final double kWristConversionFactor = 0;
    public static final double kAbsoluteWristOffset = 0;

    public static final double KPWrist = 0;
    public static final double KIWrist = 0;
    public static final double KDWrist = 0;

    public static final double kPWristMotor = 0;
    public static final double kIWristMotor = 0;
    public static final double KDWristMotor = 0;
    public static final double kTopLimit = 0;
    public static final double kTopIntakeLimit = 0;
    public static final double kBottomLimit = 0;
    public static final double kBottomIntakeLimit = 0;
    public static final double kAbsoluteWristOffest = 0;
    public static final double kSWrist = 0;
    public static final double kVWrist = 0;
    public static final double kAWrist = 0;
    public static final double kGWrist = 0;
    public static final double feedforwardVelocity = 0;
    public static final double feedforwardAcc = 0;
  }
}