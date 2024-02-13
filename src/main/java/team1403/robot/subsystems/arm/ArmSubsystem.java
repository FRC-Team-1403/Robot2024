package team1403.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

/**
 * Class creating the arm subsystem. The subsystem takes care of moving the arm
 * to a specified position.
 * The specified position can be changed via the moveArm() method.
 * 
 */
public class ArmSubsystem extends SubsystemBase {
  // Arm
  private final CANSparkMax m_leftPivotMotor;
  private final CANSparkMax m_rightPivotMotor;
  private final DutyCycleEncoder m_armAbsoluteEncoder;
  private final PIDController m_pivotPid;

  // Setpoints
  private double m_pivotAngleSetpoint;

  /**
   * Initializing the arn subsystem.
   */
  public ArmSubsystem() {
    m_leftPivotMotor = new CANSparkMax(Constants.CanBus.leftPivotMotorID, MotorType.kBrushless);
    m_rightPivotMotor = new CANSparkMax(Constants.CanBus.rightPivotMotorID, MotorType.kBrushless);
    m_armAbsoluteEncoder = new DutyCycleEncoder(Constants.RioPorts.kArmAbsoluteEncoder);

    configPivotMotors();

    m_pivotPid = new PIDController(Constants.Arm.KPArmPivot, Constants.Arm.KIArmPivot, Constants.Arm.KDArmPivot);

    this.m_pivotAngleSetpoint = getPivotAngle();    
  }

  // --------------------------- Setup methods ---------------------------

  /**
   * Configures all the motors associated with the subsystem.
   */
  private void configPivotMotors() {
    // Pivot
    m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
    m_leftPivotMotor.enableVoltageCompensation(12);
    m_leftPivotMotor.setSmartCurrentLimit(25);
    m_rightPivotMotor.follow(m_leftPivotMotor, true);
  }

  // --------------------------- Pivot Methods ---------------------------

  /**
   * Returns the position of pivot in degrees as specified by the absolute
   * encoder.
   *
   * @return The angle of the pivot in degrees.
   */
  public double getPivotAngle() {
    double value = (m_armAbsoluteEncoder.getAbsolutePosition() * 360) + Constants.Arm.kAbsolutePivotOffset;
    return value;
  }

  public PIDController getPidController() {
    return m_pivotPid;
  }

  /**
   * Moves the pivot to the desired angle.
   * 
   * @param desiredAngle the angle to move the pivot to in degrees.
   */
  private void setAbsolutePivotAngle(double desiredAngle) {
    // Feedforward
    double currentAngle = getPivotAngle();
    double normalizedCurrentAngle = currentAngle;

    // Feedback
    double feedback = m_pivotPid.calculate(normalizedCurrentAngle, desiredAngle);

    SmartDashboard.putNumber("Arm Feedback", feedback);
    double speed = MathUtil.clamp(feedback, -1, 1);
    m_leftPivotMotor.set(speed);
  }

  /**
   * Checks if the given angle is in the bounds of the pivot.
   *
   * @param angle the given angle
   * @return true if the given angle is in the bounds of the pivot.
   */
  private boolean isInPivotBounds(double angle) {
    return (angle >= Constants.Arm.kMinPivotAngle && angle <= Constants.Arm.kMaxPivotAngle);
  }

  /**
   * Limits the given angle in between the min and max pivot angles as defined in
   * the RobotConfig.Arm.
   * 
   * @param angle the angle to limit.
   * @return the limited angle.
   */
  public double limitPivotAngle(double angle) {
    return MathUtil.clamp(angle, Constants.Arm.kMinPivotAngle, Constants.Arm.kMaxPivotAngle);
  }

  /**
   * Sets the setpoints for the pivot, extension, wrist, and intake to go to.
   *
   * @param pivotAngle      the pivot angle.
   */
  public void moveArm(double pivotAngle) {
    this.m_pivotAngleSetpoint = limitPivotAngle(pivotAngle);
  }

  public void setArmSpeed(double speed) {
    m_leftPivotMotor.set(speed);
  }

  /**
   * Sets the setpoints for the pivot, extension, wrist, and intake to go to.
   *
   * @param state the setpoints for the arm to move to.
   */
  public void moveArm(ArmState state) {
    this.m_pivotAngleSetpoint = limitPivotAngle(state.armPivot);
  }

  /**
   * Returns whether the arm is at the current setpoint.
   *
   * @return true if the arm is at the current setpoint.
   */
  public boolean isAtSetpoint() {
    double currentPivotAngle = getPivotAngle();

    if (Math.abs(currentPivotAngle - this.m_pivotAngleSetpoint) > 7) {
      return false;
    }

    return true;
  }

  @Override
  public void periodic() {
    if (isInPivotBounds(this.m_pivotAngleSetpoint)) {
      setAbsolutePivotAngle(this.m_pivotAngleSetpoint);
    } else if (m_leftPivotMotor.getOutputCurrent() > Constants.Arm.kPivotMotorMaxAmperage) {
      m_leftPivotMotor.stopMotor();
    }

    // Track Values
    SmartDashboard.putNumber("Pivot Angle", getPivotAngle());
    SmartDashboard.putNumber("Pivot Setpoint", getPivotAngleSetpoint());
  }

  /**
   * Returns the object for the pivot motor.
   * 
   * @return the pivot motor.
   */
  public CANSparkMax getLeftPivotMotor() {
    return m_leftPivotMotor;
  }

  /**
   * Returns the setpoint for the pivot.
   * 
   * @return the setpoint in degrees.
   */
  public double getPivotAngleSetpoint() {
    return m_pivotAngleSetpoint;
  }
}