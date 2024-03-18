package team1403.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import team1403.lib.device.Device;
import team1403.lib.device.Encoder;
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
    private double m_absoluteEncoderResetIterations = 0;
  
    private final CougarSparkMax m_driveMotor;
    private final CougarSparkMax m_steerMotor;
  
    private final CanCoder m_absoluteEncoder;
    private final double m_absoluteEncoderOffset;
    private final Encoder m_driveRelativeEncoder;
    private final RelativeEncoder m_steerRelativeEncoder;
    private final SparkPIDController m_steerPidController;
    private final String m_name;
    private final boolean m_inverted;

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
      m_driveRelativeEncoder = m_driveMotor.getEmbeddedEncoder();
      m_absoluteEncoderOffset = offset;
      m_steerRelativeEncoder = m_steerMotor.getEncoder();
      m_steerPidController = m_steerMotor.getPIDController();
  
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

      //Config steer relative encoder
      m_steerRelativeEncoder.setPositionConversionFactor(
          Swerve.kSteerRelativeEncoderPositionConversionFactor);
      m_steerRelativeEncoder.setVelocityConversionFactor(
          Swerve.kSteerRelativeEncoderVelocityConversionFactor);
      m_steerRelativeEncoder.setPosition(getAbsoluteAngle());
    }
  
    private void initSteerMotor() {
      m_steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      m_steerMotor.setInverted(false);
      m_steerMotor.enableVoltageCompensation(Swerve.kVoltageSaturation);
      m_steerMotor.setSmartCurrentLimit(Swerve.kCurrentLimit);
  
      m_steerPidController.setP(Swerve.kPTurning);
      m_steerPidController.setI(Swerve.kITurning);
      m_steerPidController.setD(Swerve.kDTurning);
      m_steerPidController.setFeedbackDevice((MotorFeedbackSensor) m_steerRelativeEncoder);
      m_steerPidController.setPositionPIDWrappingMaxInput(180);
      m_steerPidController.setPositionPIDWrappingMinInput(-180);
      m_steerPidController.setPositionPIDWrappingEnabled(true);
    }
  
    public void initDriveMotor() {
      m_driveMotor.setInverted(m_inverted);
      m_driveMotor.setVoltageCompensation(Constants.Swerve.kVoltageSaturation);
      m_driveMotor.setSmartCurrentLimit(Constants.Swerve.kCurrentLimit);
    }
  
    /**
     * The method for getting the steer angle.
     *
     * @return The motor angles in radians.
     */
    public double getSteerAngle() {
      return normalizeAngle(m_steerRelativeEncoder.getPosition());
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
     * Normalizes angle value to be inbetween values 0 to 2pi.
     *
     * @param angle angle to be normalized
     * @return angle value between 0 to 2pi
     */
    private double normalizeAngle(double angle) {
      angle %= (2.0 * Math.PI);
      if (angle < 0.0) {
        angle += 2.0 * Math.PI;
      }
      return angle;
    }
  
    /**
     * Returns difference (targetAngle - getSteerAngle())
      normalized in range -pi .. pi
     *
     * @param targetAngle the angle to be moved to
     * @return The steer angle after accounting for error.
     */
    public double normalizeAngleError(double targetAngle) {
      // Angle is inbetween 0 to 2pi
  
      double difference = targetAngle - getSteerAngle();
      // Change the target angle so the difference is in the range [-pi, pi) instead
      // of [0, 2pi)
      if (difference >= Math.PI) {
        targetAngle -= 2.0 * Math.PI;
      } else if (difference < -Math.PI) {
        targetAngle += 2.0 * Math.PI;
      } 
      return targetAngle - getSteerAngle();
    }
  
    /**
     * Converts the steer angle to the next angle the swerve module should turn to.
     *
     * @param steerAngle the current steer angle.
     */
    private double convertSteerAngle(double steerAngle) {
      steerAngle = normalizeAngle(steerAngle);
      double difference = normalizeAngleError(steerAngle);
  
      // If the difference is greater than 90 deg or less than -90 deg the drive can
      // be inverted so the total
      // movement of the module is less than 90 deg
      if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
        // Only need to add 180 deg here because the target angle will be put back into
        // the range [0, 2pi)
        steerAngle += Math.PI;
      }
  
      // Put the target angle back into the range [0, 2pi)
      steerAngle = normalizeAngle(steerAngle);
  
      // Angle to be changed is now in radians
      double referenceAngleRadians = steerAngle;
  
      double currentAngleRadians = m_steerRelativeEncoder.getPosition();
  
      // Reset the NEO's encoder periodically when the module is not rotating.
      // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
      // fully set up, and we don't
      // end up getting a good reading. If we reset periodically this won't matter
      // anymore.
      if (m_steerRelativeEncoder.getVelocity() 
              < Swerve.kEncoderResetMaxAngularVelocity) {
        if (++m_absoluteEncoderResetIterations >= Swerve.kEncoderResetIterations) {
          m_absoluteEncoderResetIterations = 0;
          double absoluteAngle = getAbsoluteAngle();
          m_steerRelativeEncoder.setPosition(getAbsoluteAngle());
          currentAngleRadians = absoluteAngle;
        }
      } else {
        m_absoluteEncoderResetIterations = 0;
      }
  
      double currentAngleRadiansMod = normalizeAngle(currentAngleRadians);
  
      // The reference angle has the range [0, 2pi)
      // but the Falcon's encoder can go above that
      double adjustedReferenceAngleRadians = referenceAngleRadians 
          + currentAngleRadians - currentAngleRadiansMod;
      if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
        adjustedReferenceAngleRadians -= 2.0 * Math.PI;
      } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
        adjustedReferenceAngleRadians += 2.0 * Math.PI;
      }
  
      // The position that the motor should turn to
      // when taking into account the ticks of the motor
      return adjustedReferenceAngleRadians;
    }

    /**
     * Method for setting the drive voltage and steering angle.
     *
     * @param driveMetersPerSecond driving meters per second.
     * @param steerAngle           steering angle.
     *
     */
    public void set(double driveMetersPerSecond, double steerAngle) {
      // Set driveMotor according to percentage output
      this.m_driveMotor.set(driveMetersPerSecond);
  
      // Set steerMotor according to position of encoder
      this.m_steerMotor.getPIDController()
          .setReference(convertSteerAngle(steerAngle), CANSparkMax.ControlType.kPosition);
    }
  
    /**
     * Gets the current angle reading of the encoder in radians.
     *
     * @return The current angle in radians. Range: [0, 2pi)
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
    public Encoder getDriveRelativeEncoder() {
      return m_driveRelativeEncoder;
    }
  
    /**
     * Returns the SwerveModulePosition of this particular module.
     *
     * @return the SwerveModulePosition, which represents the distance 
     *         travelled and the angle of the module.
     */
    public SwerveModulePosition getModulePosition() {
      return new SwerveModulePosition(m_driveRelativeEncoder.getPositionValue(), 
            new Rotation2d(getSteerAngle()));
    }
    
    /**
     * Returns the current velocity of the drive motor.
     *
     * @return the current velocity of the drive motor
     */
    public double getDriveVelocity() {
      return m_driveRelativeEncoder.getVelocityValue();
    }
  
    /**
     * Returns the current state of the swerve module as defined by 
     * the relative encoders of the drive and steer motors.
     *
     * @return the current state of the swerve module
     */
    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
    }
  
    /**
     * Returns the current position of the swerve module as defined by
     * the relative encoders of the drive and steer motors.
     *
     * @return the current position of the swerve module
     */
    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(m_driveRelativeEncoder.getPositionValue(),
          new Rotation2d(getSteerAngle()));
    }
  }