package team1403.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
  // lead motor
  private final CANSparkMax m_leftMotor;
  // following motor
  private final CANSparkMax m_rightMotor;
  private final DutyCycleEncoder m_encoder;
  private final ProfiledPIDController m_armPid;
  private final ArmFeedforward m_feedforward;

  private double tempFF;

  // Setpoints
  private double m_angleSetpoint;

  private boolean m_currentLimitTripped = false;

  /**
   * Initializing the arn subsystem.
   */
  public ArmSubsystem() {
    m_feedforward = new ArmFeedforward(0, Constants.Arm.kFeedforwardG, Constants.Arm.kFeedforwardV);
    m_leftMotor = new CANSparkMax(Constants.CanBus.leftPivotMotorID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.CanBus.rightPivotMotorID, MotorType.kBrushless);
    m_encoder = new DutyCycleEncoder(Constants.RioPorts.kArmAbsoluteEncoder);

    configPivotMotors();

    m_armPid = new ProfiledPIDController(Constants.Arm.KPArmPivot, Constants.Arm.KIArmPivot, Constants.Arm.KDArmPivot, new TrapezoidProfile.Constraints(370, 570));
    m_armPid.reset(getPivotAngle(), 0);

    this.m_angleSetpoint = getPivotAngle();
  }

  // --------------------------- Setup methods ---------------------------

  /**
   * Configures all the motors associated with the subsystem.
   */
  private void configPivotMotors() {
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.enableVoltageCompensation(Constants.Arm.kPivotMotorVoltageLimit);
    m_leftMotor.setSmartCurrentLimit(Constants.Arm.kPivotMotorCurrentLimit);
    m_rightMotor.setSmartCurrentLimit(Constants.Arm.kPivotMotorCurrentLimit);
    m_rightMotor.follow(m_leftMotor, true);
  }

  // --------------------------- Pivot Methods ---------------------------

  /**
   * Returns the position of pivot in degrees as specified by the absolute
   * encoder.
   *
   * @return The angle of the pivot in degrees.
   */
  public double getPivotAngle() {
    double value = (m_encoder.getAbsolutePosition() * 360) + Constants.Arm.kAbsolutePivotOffset;
    return value;
  }

  public double getSetpoint(){
    return m_angleSetpoint;
  }

  /**
   * Moves the pivot to the desired angle.
   * 
   * @param desiredAngle the angle to move the pivot to in degrees.
   */
  private void setAbsolutePivotAngle(double desiredAngle) {
    //Feedback
    double feedback = m_armPid.calculate(getPivotAngle(), desiredAngle);

    Logger.recordOutput("Arm speed", feedback);
    setArmSpeed(feedback);
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
    this.m_angleSetpoint = limitPivotAngle(pivotAngle);
  }

  public void setArmSpeed(double speed) {
    double ff = m_feedforward.calculate(Units.degreesToRadians(getPivotAngle() - 106.4), 0);
    if (isOverUpperBound()) m_leftMotor.set(-0.1);
    else if (isUnderLowerBound()) m_leftMotor.set(0.1);
    else m_leftMotor.set(MathUtil.clamp(speed + ff, -0.7, 0.8));
  }

  /**
   * Sets the setpoints for the pivot, extension, wrist, and intake to go to.
   *
   * @param state the setpoints for the arm to move to.
   */
  public void moveArm(ArmState state) {
    this.m_angleSetpoint = limitPivotAngle(state.armPivot);
  }

  /**
   * Returns whether the arm is at the current setpoint.
   *
   * @return true if the arm is at the current setpoint.
   */
  public boolean isAtSetpoint() {
    return Math.abs(getPivotAngle() - m_angleSetpoint) <= 5.0;
  }

  public boolean isOverUpperBound(){
    return (getPivotAngle() >= Constants.Arm.kMaxPivotAngle);
  }

   public boolean isUnderLowerBound(){
    return (getPivotAngle() <= Constants.Arm.kMinPivotAngle);
  }

  @Override
  public void periodic() {
    if (isInPivotBounds(this.m_angleSetpoint)) {
      setAbsolutePivotAngle(this.m_angleSetpoint);
    } else if (m_leftMotor.getOutputCurrent() > Constants.Arm.kPivotMotorMaxAmperage) {
      m_currentLimitTripped = true;
      m_leftMotor.stopMotor();
    }
    

    // Track Values
    SmartDashboard.putBoolean("Arm IsAtSetpoint", isAtSetpoint());
    Logger.recordOutput("Pivot Angle", getPivotAngle());
    Logger.recordOutput("Pivot Setpoint", getPivotAngleSetpoint());
    Logger.recordOutput("Pivot Voltage", m_leftMotor.getBusVoltage());
    Logger.recordOutput("Pivot Speed", m_leftMotor.get());
    Logger.recordOutput("Arm Current Trip", m_currentLimitTripped);
  }

  /**
   * Returns the object for the pivot motor.
   * 
   * @return the pivot motor.
   */
  public CANSparkMax getLeftPivotMotor() {
    return m_leftMotor;
  }

  /**
   * Returns the setpoint for the pivot.
   * 
   * @return the setpoint in degrees.
   */
  public double getPivotAngleSetpoint() {
    return m_angleSetpoint;
  }
}