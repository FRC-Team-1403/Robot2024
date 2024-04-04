package team1403.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.device.Device;
import team1403.lib.device.wpi.CanCoder;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;
import team1403.robot.Constants.Swerve;

/**
 * Represents a swerve module. Consists of a drive motor, steer motor, 
 * and their respective relative encoders.
 * Also consists of a absolute encoder to track steer angle.
 */
public class SwerveModule implements Device {
    private final CougarSparkMax m_driveMotor;
    private final CougarSparkMax m_steerMotor;

    private final CanCoder m_absoluteEncoder;
    private final double m_absoluteEncoderOffset;
    private final RelativeEncoder m_driveRelativeEncoder;
    private final PIDController m_steerPidController;
    private final SparkPIDController m_drivePIDController;
    private final String m_name;
    private final boolean m_inverted;
    private double m_targetVelocity;
    private double m_targetSteerAngle;


    /**
     * Swerve Module represents a singular swerve module for a
     * swerve drive train.
     *
     * <p>Each swerve module consists of a drive motor,
     * changing the velocity of the wheel, and a steer motor, changing
     * the angle of the actual wheel inside of the module.
     *
     * <p>The swerve module also features
     * an absolute encoder to ensure the angle of
     * the module is always known, regardless if the bot is turned off
     * or not.
     *
     */
    public SwerveModule(String name, int driveMotorPort, int steerMotorPort,
        int canCoderPort, double offset) {
      this(name, driveMotorPort, steerMotorPort, canCoderPort, offset, true);
    }

    public SwerveModule(String name, int driveMotorPort, int steerMotorPort,
    int canCoderPort, double offset, boolean inverted)
    {
      m_inverted = inverted;
      m_name = name;

      m_driveMotor = CougarSparkMax.makeBrushless("DriveMotor", driveMotorPort,
          SparkRelativeEncoder.Type.kHallSensor);
      m_steerMotor = CougarSparkMax.makeBrushless("SteerMotor", steerMotorPort,
          SparkRelativeEncoder.Type.kHallSensor);
      m_absoluteEncoder = new CanCoder("CanCoder", canCoderPort);
      m_driveRelativeEncoder = m_driveMotor.getEncoder();
      m_absoluteEncoderOffset = offset;
      m_steerPidController = new PIDController(Swerve.kPTurning, Swerve.kITurning, Swerve.kDTurning);
      m_drivePIDController = m_driveMotor.getPIDController();
      m_steerPidController.enableContinuousInput(0, 2*Math.PI);
      m_targetSteerAngle = 0;

      initEncoders();
      initSteerMotor();
      initDriveMotor();
    }

    @Override
    public String getName() {
      return m_name;
    }

    public void initEncoders() {
      // Config absolute encoder
      if (m_absoluteEncoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Green) {
        System.err.println("CANCoder magnetic field strength is unacceptable.");
      }
      MagnetSensorConfigs magnetSensor = new MagnetSensorConfigs();
      //in units of rotations
      magnetSensor.MagnetOffset = (m_absoluteEncoderOffset)/(2*Math.PI);
      magnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
      magnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

      CANcoderConfiguration config = new CANcoderConfiguration().withMagnetSensor(magnetSensor);

      m_absoluteEncoder.getConfigurator().apply(config, 0.250);

      //avoid overrun, and get more up to date values for PID
      //m_absoluteEncoder.getPosition().setUpdateFrequency(500, 0.002);

      //m_absoluteEncoder.setPositionToAbsolute();
      //m_absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250);

      // YAGSL website includes conversion factors for MK4 L3 drive, so instead of calcluating, 
      // we are are using their number
      // double drivePositionConversionFactor = Math.PI * Swerve.kWheelDiameterMeters 
      //       * Swerve.kDriveReduction (gear ratios);

      m_driveRelativeEncoder.setPositionConversionFactor(Constants.Swerve.kDrivePositionConversionFactor);
      // Set velocity in terms of seconds
      m_driveRelativeEncoder.setVelocityConversionFactor(Constants.Swerve.kDrivePositionConversionFactor / 60.0);

      m_absoluteEncoder.setPositionConversionFactor(2 * Math.PI);
      m_absoluteEncoder.setVelocityConversionFactor(2 * Math.PI);
    }

    private void initSteerMotor() {
      m_steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      m_steerMotor.setInverted(false);
      m_steerMotor.enableVoltageCompensation(Swerve.kVoltageSaturation);
      m_steerMotor.setSmartCurrentLimit(Swerve.kSteerCurrentLimit);
    }

    public void initDriveMotor() {
      m_driveMotor.setInverted(m_inverted);
      m_driveMotor.setVoltageCompensation(Constants.Swerve.kVoltageSaturation);
      m_driveMotor.setSmartCurrentLimit(Constants.Swerve.kDriveCurrentLimit);

      m_drivePIDController.setP(Constants.Swerve.kPDrive);
      m_drivePIDController.setI(Constants.Swerve.kIDrive);
      m_drivePIDController.setD(Constants.Swerve.kDDrive);
      m_drivePIDController.setFF(1.0/Swerve.kMaxSpeed);
      m_drivePIDController.setFeedbackDevice((MotorFeedbackSensor)m_driveRelativeEncoder);
      m_drivePIDController.setOutputRange(-1,1);
      // //slot 0 is used by default
      // m_drivePIDController.setSmartMotionMaxVelocity(6000, 0);
      // m_drivePIDController.setSmartMotionMinOutputVelocity(0, 0);
      // m_drivePIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

      // m_driveMotor.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Sets the contoller mode.
     *
     * @param mode its the mode for the controller.
     */
    public void setControllerMode(CANSparkMax.IdleMode mode) {
      m_driveMotor.setIdleMode(mode);
    }
  
    /**
     * Sets the ramp rate.
     *
     * @param rate speed in seconds motor will take to ramp to speed
     */
    public void setRampRate(double rate) {
      m_driveMotor.setOpenLoopRampRate(rate);
    }

    /**
     * Normalizes angle value to be inbetween values -pi to pi.
     *
     * @param angle angle to be normalized
     * @return angle value between -pi to pi
     */
    private double normalizeAngle(double angle) {
      angle %= (2.0 * Math.PI);
      if (angle < 0.0) {
        angle += 2.0 * Math.PI;
      }
      return angle;
    }

    /**
     * Method for setting the drive voltage and steering angle.
     *
     * @param driveMetersPerSecond driving meters per second.
     * @param steerAngle           steering angle.
     *
     */
    public void set(double driveMetersPerSecond, double steerAngle) {
      // Set driveMotor according to velocity input
      // System.out.println("drive input speed: " + driveMetersPerSecond);
      this.m_drivePIDController.setReference(driveMetersPerSecond, ControlType.kVelocity);
      m_targetVelocity = driveMetersPerSecond;

      // Set steerMotor according to position of encoder
      m_targetSteerAngle = normalizeAngle(steerAngle);
    }

    /**
     * Gets the current angle reading of the encoder in radians.
     *
     * @return The current angle in radians. Range: [-pi, pi)
     */
    public double getAbsoluteAngle() {
      return normalizeAngle(m_absoluteEncoder.getPositionValue());
    }
  
    /**
     * Returns the drive motor object associated with the module.
     *
     * @return the drive motor object.
     * 
     */
    public CANSparkMax getDriveMotor() {
      return m_driveMotor;
    }
  
    /**
     * Returns the steer motor object associated with the module.
     *
     * @return the steer motor object.
     * 
     */
    public CANSparkMax getSteerMotor() {
      return m_steerMotor;
    }
  
    /**
     * Returns the CANCoder object associated with the module.
     *
     * @return the CANCoder object.
     * 
     */
    public CanCoder getAbsoluteEncoder() {
      return m_absoluteEncoder;
    }
  
    /**
     * Returns the relative encoder for the drive motor.
     *
     * @return relative encoder value from drive motor.
     *
     */
    public RelativeEncoder getDriveRelativeEncoder() {
      return m_driveRelativeEncoder;
    }
  
    /**
     * Returns the SwerveModulePosition of this particular module.
     *
     * @return the SwerveModulePosition, which represents the distance 
     *         travelled and the angle of the module.
     */
    public SwerveModulePosition getModulePosition() {
      return new SwerveModulePosition(m_driveRelativeEncoder.getPosition(),
            new Rotation2d(getAbsoluteAngle()));
    }
    
    /**
     * Returns the current velocity of the drive motor.
     *
     * @return the current velocity of the drive motor
     */
    public double getDriveVelocity() {
      return m_driveRelativeEncoder.getVelocity();
    }
  
    /**
     * Returns the current state of the swerve module as defined by 
     * the relative encoders of the drive and steer motors.
     *
     * @return the current state of the swerve module
     */
    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteAngle()));
    }

    public void periodic() {
      m_steerMotor.set(m_steerPidController.calculate(getAbsoluteAngle(), m_targetSteerAngle));
      SmartDashboard.putNumber(getName() + " current angle", getAbsoluteAngle());
      SmartDashboard.putNumber(getName() + " desired angle", m_steerPidController.getSetpoint());
      SmartDashboard.putNumber(getName() + " current velocity", m_driveMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber(getName() + " desired velocity", m_targetVelocity);
    }
  }