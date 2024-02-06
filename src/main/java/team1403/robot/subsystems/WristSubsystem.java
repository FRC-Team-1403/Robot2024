package team1403.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;
import team1403.robot.Constants.Arm;

public class WristSubsystem {
  private final CANSparkMax m_wristMotor;
  private final DutyCycleEncoder m_wristAbsoluteEncoder;
  private double m_wristAngleSetpoint;

  public WristSubsystem() {
    m_wristMotor = new CANSparkMax(Constants.CanBus.wristMotor, MotorType.kBrushless);
    m_wristAbsoluteEncoder = new DutyCycleEncoder(Constants.RioPorts.kWristAbsoluteEncoder);

    configWristMotor();
    configEncoders();

    this.m_wristAngleSetpoint = getAbsoluteWristAngle();
  }

  /**
   * Configures all the encoders associated with the susbystem.
   */
  private void configEncoders() {
    // Wrist encoders
    m_wristMotor.getEncoder().setPositionConversionFactor(Constants.Wrist.kWristConversionFactor);
    new Thread(() -> {
      try {
        Thread.sleep(2000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      double wristAngle = getAbsoluteWristAngle();
      m_wristMotor.getEncoder().setPosition(wristAngle);
      m_wristAngleSetpoint = wristAngle;
    }).start();
  }

  /**
   * Configures all the motors associated with the subsystem.
   */
  private void configWristMotor() {
    // Wrist
    PIDController wristController = new PIDController(Constants.Wrist.KPWrist, Constants.Wrist.KIWrist, Constants.Wrist.KDWrist);
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setInverted(false);
    m_wristMotor.enableVoltageCompensation(12);
    m_wristMotor.setSmartCurrentLimit(20);

    wristController.setP(Constants.Wrist.kPWristMotor);
    wristController.setI(Constants.Wrist.kIWristMotor);
    wristController.setD(Constants.Wrist.KDWristMotor);
  }

  /**
   * Gets the absolute wrist encoder value.
   *
   * @return The absolute encoder value of the wrist.
   */
  public double getAbsoluteWristAngle() {
    double value = (m_wristAbsoluteEncoder.getAbsolutePosition() * 360) + Constants.Wrist.kAbsoluteWristOffset;

    if (value < 0) {
      value += 360;
    }
    if (value > 360) {
      value -= 360;
    }
    return value;
  }

  /**
   * Moves the wrist to the given angle.
   * 
   * @param absoluteWristAngle the angle to move the wrist to.
   */
  private void setAbsoluteWristAngle(double absoluteWristAngle) {
    m_wristMotor.getPIDController().setReference(absoluteWristAngle,
        CANSparkMax.ControlType.kPosition);
  }

  /**
   * Limits the wrist angle between the min and max wrist angles as defined in
   * RobotConfig.Arm.
   * 
   * @param angle the given angle to limit.
   * @return the limited angle.
   */
  public double limitWristAngle(double angle) {
    return MathUtil.clamp(angle, Constants.Wrist.kMinWristAngle, Constants.Wrist.kMaxWristAngle);
  }

  /**
   * Checks if the given angle is in the bounds of the wrist.
   *
   * @param angle the given angle
   * @return true if the given angle is in the bounds of the wrist.
   */
  private boolean isInWristBounds(double angle) {
    return (angle > Constants.Wrist.kMinWristAngle && angle < Constants.Wrist.kMaxWristAngle);
  }

  /**
   * Sets the setpoints for the pivot, extension, wrist, and intake to go to.
   *
   * @param wristAngle      the wrist angle relative to the pivot.
   */
  public void moveArm(double wristAngle) {
    this.m_wristAngleSetpoint = limitWristAngle(wristAngle);
  }

  /**
   * Sets the setpoints for the pivot, extension, wrist, and intake to go to.
   *
   * @param state the setpoints for the arm to move to.
   */
  public void moveArm(ArmState state) {
    this.m_wristAngleSetpoint = limitWristAngle(state.wristAngle);
  }

  /**
   * Returns whether the arm is at the current setpoint.
   *
   * @return true if the arm is at the current setpoint.
   */
  public boolean isAtSetpoint() {
    double currentWristAngle = getAbsoluteWristAngle();

    if (Math.abs(currentWristAngle - this.m_wristAngleSetpoint) > 6) {
      return false;
    }

    return true;
  }

  @Override
  public void periodic() {
    
    // Wrist
      if (isInWristBounds(m_wristMotor.getEncoder().getPosition())
          || isInWristBounds(this.m_wristAngleSetpoint)) {
        setAbsoluteWristAngle(this.m_wristAngleSetpoint);
      } else if (m_wristMotor.getOutputCurrent() > 25) {
        setAbsoluteWristAngle(m_wristMotor.getEncoder().getPosition());
      } else {
        setAbsoluteWristAngle(m_wristMotor.getEncoder().getPosition());
      }

    // Track Values
    SmartDashboard.putNumber("Wrist Angle", getAbsoluteWristAngle());
    SmartDashboard.putNumber("WristSetpoint", getWristAngleSetpoint());
  }
  
  /**
   * Returns the object for the wrist motor.
   * 
   * @return the wrist motor.
   */
  public CANSparkMax getWristMotor() {
    return m_wristMotor;
  }

  /**
   * Returns the setpoint for the wrist.
   * 
   * @return the setpoint in degrees.
   */
  public double getWristAngleSetpoint() {
    return m_wristAngleSetpoint;
  }
}