package team1403.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;
import team1403.robot.RobotContainer;
import team1403.robot.Constants.Arm;
import team1403.robot.Constants.CanBus;

/**
 * Class creating the arm subsystem. The subsystem takes care of moving the arm
 * to a specified position.
 * The specified position can be changed via the moveArm() method.
 * 
 */
public class ArmSubsystem extends SubsystemBase {
  // Wrist
  private final CougarSparkMax m_wristMotor;
  private final DutyCycleEncoder m_wristAbsoluteEncoder;
  private final PIDController m_wristController;

  // Arm
  private final CANSparkMax m_leftPivotMotor;
  private final CANSparkMax m_rightPivotMotor;
  private final AnalogEncoder m_armAbsoluteEncoder;
  private final PIDController m_pivotPid;
  private final WpiLimitSwitch m_maxArmLimitSwitch;

  private boolean previousLimitSwitchTrigger = true;

  // Intake
  private final CANSparkMax m_intakeMotor;

  // Telescope
  private final CANSparkMax m_extensionMotor;
  private final DigitalInput m_minMagneticSwitch;
  private final DigitalInput m_maxMagneticSwitch;
  private boolean m_extensionLimitSwitchReset = false;
  private boolean m_ignoreExtensionLimit = false;

  // Setpoints
  private double m_wristAngleSetpoint;
  private double m_pivotAngleSetpoint;
  private double m_intakeSpeedSetpoint;
  private double m_extensionLengthSetpoint;
  private double m_intakePosition;

  /**
   * Initializing the arn subsystem.
   *
   * @param injectedParameters Cougar injected parameters.
   */
  public ArmSubsystem() {
    m_wristMotor = CougarSparkMax.makeBrushless("wrist Motor",Constants.CanBus.wristMotor, SparkRelativeEncoder.Type.kHallSensor);
    m_wristAbsoluteEncoder = new DutyCycleEncoder(1);

    m_leftPivotMotor = new CANSparkMax(Constants.CanBus.leftPivotMotorID, MotorType.kBrushless);
    m_rightPivotMotor = new CANSparkMax(Constants.CanBus.rightPivotMotorID, MotorType.kBrushless);
    m_armAbsoluteEncoder = new AnalogEncoder(Constants.RioPorts.kArmAbsoluteEncoder);

    m_intakeMotor = new CANSparkMax(Constants.CanBus.intakeAndShooterMotorTop, MotorType.kBrushed);

    m_extensionMotor = new CANSparkMax(Constants.CanBus.telescopicArmMotor, MotorType.kBrushless);

    m_maxArmLimitSwitch = new WpiLimitSwitch("maxArmLimitSwitch",
        Constants.RioPorts.kArmLimitSwitch);

    configWristMotor();
    configEncoders();

    m_wristController = new PIDController(0.05, 0, 0);

    m_pivotPid = new PIDController(Constants.Arm.KPArmPivot, Constants.Arm.KIArmPivot, Constants.Arm.KDArmPivot);
    m_minMagneticSwitch = new DigitalInput(Constants.RioPorts.kExtensionMinMagneticSwitch);
    m_maxMagneticSwitch = new DigitalInput(Constants.RioPorts.kExtensionMaxMagneticSwitch);

    Constants.Arm.kAbsolutePivotOffset = 0;
    double difference = Constants.Arm.kMaxPivotAngle - getAbsolutePivotAngle();
    Constants.Arm.kAbsolutePivotOffset = difference;

    this.m_pivotAngleSetpoint = getAbsolutePivotAngle();
    this.m_wristAngleSetpoint = getAbsoluteWristAngle();
    this.m_extensionLengthSetpoint = getExtensionLength();

    
  }

  // --------------------------- Setup methods ---------------------------

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

    // Telescopic encoders
    m_extensionMotor.getEncoder().setPositionConversionFactor(
        Constants.Arm.kExtensionConversionFactor);

    // Arm encoders
    m_leftPivotMotor.getEncoder().setPositionConversionFactor(1.53285964552);
    m_leftPivotMotor.getEncoder().setPosition(getAbsolutePivotAngle());
  }

  /**
   * Configures all the motors associated with the subsystem.
   */
  private void configWristMotor() {
    // Wrist
    final SparkPIDController wristController = m_wristMotor.getPIDController();
    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_wristMotor.setInverted(false);
    m_wristMotor.enableVoltageCompensation(12);
    m_wristMotor.setSmartCurrentLimit(20);
    m_wristMotor.setRampRate(0.25);

    wristController.setP(0.05);
    wristController.setI(Constants.Wrist.KIWrist);
    wristController.setD(0);
    wristController.setFeedbackDevice((MotorFeedbackSensor) m_wristMotor.getEncoder());
    wristController.setPositionPIDWrappingEnabled(false);

    // Pivot
    m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
    m_leftPivotMotor.enableVoltageCompensation(12);
    m_leftPivotMotor.setSmartCurrentLimit(25);
    m_rightPivotMotor.follow(m_leftPivotMotor, true);

    // intake
    final SparkPIDController intakeMotorController = m_intakeMotor.getPIDController();
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_intakeMotor.enableVoltageCompensation(12);
    m_intakeMotor.setSmartCurrentLimit(20);

    intakeMotorController.setP(Constants.Intake.kPIntake);
    intakeMotorController.setI(Constants.Intake.kIIntake);
    intakeMotorController.setD(Constants.Intake.kDIntake);
    intakeMotorController.setFeedbackDevice(m_intakeMotor.getAlternateEncoder(1024));
    intakeMotorController.setPositionPIDWrappingEnabled(false);

    // Extension
    final SparkPIDController extensionController = m_extensionMotor.getPIDController();
    m_extensionMotor.setIdleMode(IdleMode.kBrake);
    m_extensionMotor.enableVoltageCompensation(12);
    m_extensionMotor.setSmartCurrentLimit(20);
    m_extensionMotor.setOpenLoopRampRate(0.25);

    extensionController.setP(Constants.Arm.kPArmExtension);
    extensionController.setI(Constants.Arm.kIArmExtension);
    extensionController.setD(Constants.Arm.kDArmExtension);
    extensionController.setFeedbackDevice(m_extensionMotor.getEncoder());
    extensionController.setPositionPIDWrappingEnabled(false);
  }

  // --------------------------- Wrist Methods ---------------------------

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
    // m_wristMotor.setSpeed(m_wristController.calculate(getAbsoluteWristAngle(), absoluteWristAngle));
  }

  /**
   * Limits the wrist angle between the min and max wrist angles as defined in
   * Constants.Arm.
   * 
   * @param angle the given angle to limit.
   * @return the limited angle.
   */
  public double limitWristAngle(double angle) {
    return MathUtil.clamp(angle, Constants.Wrist.kBottomLimit, Constants.Wrist.kTopLimit);
  }

  /**
   * Checks if the given angle is in the bounds of the wrist.
   *
   * @param angle the given angle
   * @return true if the given angle is in the bounds of the wrist.
   */
  private boolean isInWristBounds(double angle) {
    return (angle > Constants.Wrist.kBottomLimit && angle < Constants.Wrist.kTopLimit);
  }

  // --------------------------- Pivot Methods ---------------------------

  /**
   * Returns whether the arm limit switch has been triggered.
   *
   * @return true if the limit switch has been triggered.
   */
  public boolean isArmSwitchActive() {
    return m_maxArmLimitSwitch.isTriggered();
  }

  /**
   * Rezeros the pivot encoder. Only use when the pivot limit switch is active.
   * Accounts for any belt skipping.
   */
  private void rezeroPivot() {
    Constants.Arm.kAbsolutePivotOffset = 0;
    double difference = Constants.Arm.kMaxPivotAngle - getAbsolutePivotAngle();
    Constants.Arm.kAbsolutePivotOffset += difference;
  }

  /**
   * Returns the position of pivot in degrees as specified by the absolute
   * encoder.
   *
   * @return The angle of the pivot in degrees.
   */
  public double getAbsolutePivotAngle() {
    double value = (m_armAbsoluteEncoder.getAbsolutePosition() * 360) + Constants.Arm.kAbsolutePivotOffset;

    if (value < 0) {
      value += 360;
    }
    if (value > 360) {
      value -= 360;
    }
    return value;
  }

  /**
   * Moves the pivot to the desired angle.
   * 
   * @param desiredAngle the angle to move the pivot to in degrees.
   */
  public void setAbsolutePivotAngle(double desiredAngle) {
    // Feedforward
    double currentAngle = getAbsolutePivotAngle();
    double normalizedCurrentAngle = currentAngle;
    while (normalizedCurrentAngle > 90) {
      normalizedCurrentAngle -= 90;
    }
    double gravityCompensationFactor = ((.0004/23.128) * getExtensionLength() + .0009)
     * Constants.Arm.kBaseArmLength; //0.0009
    double feedforward = gravityCompensationFactor;
        // * Math.cos(Math.toRadians(normalizedCurrentAngle));
    if ((currentAngle < 90 && currentAngle > 0) || (currentAngle > 270 && currentAngle < 360)) {
      feedforward *= -1;
    }

    // Feedback
    double feedback = -1 * m_pivotPid.calculate(currentAngle, desiredAngle);

    if (isArmSwitchActive()) {
      feedforward = 0;
    }

    SmartDashboard.putNumber("Arm Feedforward", feedforward);
    SmartDashboard.putNumber("Arm Feedback", feedback);
    double speed = MathUtil.clamp(feedforward + feedback, -1, 1);
    m_leftPivotMotor.set(speed);
  }

  /**
   * Checks if the given angle is in the bounds of the pivot.
   *
   * @param angle the given angle
   * @return true if the given angle is in the bounds of the pivot.
   */
  private boolean isInPivotBounds(double angle) {
    return (angle >= Arm.kMinPivotAngle && angle <= Arm.kMaxPivotAngle);
  }

  /**
   * Limits the given angle in between the min and max pivot angles as defined in
   * the Constants.Arm.
   * 
   * @param angle the angle to limit.
   * @return the limited angle.
   */
  public double limitPivotAngle(double angle) {
    return MathUtil.clamp(angle, Arm.kMinPivotAngle, Arm.kMaxPivotAngle);
  }

  // --------------------------- Intake ---------------------------

  /**
   * Runs the intake at the given speed. If the intake is not running, it will
   * hold its position.
   * 
   * @param intakeSpeed the speed to run the intake at (between -1 and 1,
   *                    inclusive).
   */
  public void runIntake(double intakeSpeed) {
    if (intakeSpeed == 0) {
      m_intakePosition = m_intakeMotor.getAlternateEncoder(1024).getPosition();
      m_intakeMotor.getPIDController().setReference(m_intakePosition, CANSparkMax.ControlType.kPosition);
    } else {
      m_intakeMotor.set(m_intakeSpeedSetpoint);
    }
  }

  // --------------------------- Extension ---------------------------

  /**
   * Returns the amount the arm has extended by in inches.
   * 
   * @return the extension length of the arm in inches.
   */
  public double getExtensionLength() {
    return m_extensionMotor.getEncoder().getPosition();
  }

  /**
   * Extends the arm to the given extension length.
   * 
   * @param extensionLength the length to extend the arm to in inches.
   */
  private void setMotorExtensionLength(double extensionLength) {
    m_extensionMotor.getPIDController().setReference(
        extensionLength, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Returns whether the minimum extension limit switch is active. If it is, the
   * arm has extended 0 inches.
   * 
   * @return whether the extension limit switch is active.
   */
  private boolean isExtensionMinSwitchActive() {
    return m_minMagneticSwitch.get();
  }

  /**
   * Returns whether the maximum extension limit switch is active. If it is, the
   * arm has extended 0 inches.
   * 
   * @return whether the extension limit switch is active.
   */
  private boolean isExtensionMaxSwitchActive() {
    return m_maxMagneticSwitch.get();
  }

  /**
   * Limits the given length within bounds of the min and max extension specified
   * in Constants.Arm.
   * 
   * @param length the given length to limit.
   * @return the limited length.
   */
  public double limitExtensionLength(double length) {
    return MathUtil.clamp(length, Arm.kMinArmExtension, Arm.kMaxArmExtension);
  }

  /**
   * Checks if the given angle is in the bounds of the wrist.
   *
   * @param angle the given angle
   * @return true if the given angle is in the bounds of the wrist.
   */
  private boolean isInExtensionBounds(double length) {
    return (length >= Constants.Arm.kMinArmExtension && length <= Constants.Arm.kMaxArmExtension);
  }

  /**
   * Calculates the theoretical max arm length of the arm given the arm angle.
   *
   * @param absoluteArmAngle arm angle relative to ground
   * @return the theoretical arm length
   */
  public double theoreticalExtensionLength(double absoluteArmAngle, double height) {
    return (height / Math.cos(Math.toRadians(270 - absoluteArmAngle)))
        - Constants.Arm.kExtensionOffset - Constants.Arm.kBaseArmLength;
  }

  /**
   * Dynamically limits the extensions of arm so that it doesn't hit the ground.
   *
   * @param extensionLength extension length.
   * @return the arm length.
   */
  public double dynamicExtensionLimit(double extensionLength) {
    if (m_ignoreExtensionLimit) {
      return extensionLength;
    }

    if (getAbsolutePivotAngle() >= Constants.Arm.kFrameClearanceAngle) {
      return 0;
    } else if (getAbsolutePivotAngle() > Constants.Arm.kHorizonAngle
        && getAbsolutePivotAngle() <= Constants.Arm.kFrameClearanceAngle) {
      double maxLength = theoreticalExtensionLength(
          getAbsolutePivotAngle(), Constants.kHeightFromGround);
      return MathUtil.clamp(extensionLength, 0, maxLength);
    } else if (getAbsolutePivotAngle() > Constants.Arm.kFrameClearanceAngle
        && getAbsolutePivotAngle() <= Constants.Arm.kFrameAngle) {
      double maxLength = -(8.355 / 4) * (getAbsolutePivotAngle() - 229);
      return MathUtil.clamp(extensionLength, 0, maxLength);
    }
    return extensionLength;
  }

  public void ignoreExtensionLimit(boolean ignore) {
    m_ignoreExtensionLimit = ignore;
  }

  // --------------------------- General methods ---------------------------

  /**
   * Sets the setpoints for the pivot, extension, wrist, and intake to go to.
   *
   * @param wristAngle      the wrist angle relative to the pivot.
   * @param intakeSpeed     intake speed.
   * @param pivotAngle      the pivot angle.
   * @param extensionLength the extension length.
   */
  public void moveArm(double wristAngle, double intakeSpeed,
      double pivotAngle, double extensionLength) {
    this.m_wristAngleSetpoint = limitWristAngle(wristAngle);
    this.m_intakeSpeedSetpoint = intakeSpeed;
    this.m_pivotAngleSetpoint = limitPivotAngle(pivotAngle);
    this.m_extensionLengthSetpoint = limitExtensionLength(extensionLength);
  }

  public void moveWrist(double wristAngle) {
    this.m_wristAngleSetpoint = limitWristAngle(wristAngle);
  }

  public void moveArm(double intakeSpeed,
      double pivotAngle, double extensionLength) {
    this.m_intakeSpeedSetpoint = intakeSpeed;
    this.m_pivotAngleSetpoint = limitPivotAngle(pivotAngle);
    this.m_extensionLengthSetpoint = limitExtensionLength(extensionLength);
  }
  /**
   * Sets the setpoints for the pivot, extension, wrist, and intake to go to.
   *
   * @param state the setpoints for the arm to move to.
   */
  public void moveArm(ArmState state) {
    this.m_wristAngleSetpoint = limitWristAngle(state.wristAngle);
    this.m_intakeSpeedSetpoint = state.intakeSpeed;
    this.m_pivotAngleSetpoint = limitPivotAngle(state.armPivot);
    this.m_extensionLengthSetpoint = limitExtensionLength(state.armLength);
  }

  /**
   * Returns whether the arm is at the current setpoint.
   *
   * @return true if the arm is at the current setpoint.
   */
  public boolean isAtSetpoint() {
    double currentPivotAngle = getAbsolutePivotAngle();
    double currentWristAngle = m_wristMotor.getEncoder().getPosition();
    double currentExtensionLength = getExtensionLength();

    if (Math.abs(currentPivotAngle - this.m_pivotAngleSetpoint) > 7) {
      return false;
    }

    if (Math.abs(currentWristAngle - this.m_wristAngleSetpoint) > 6) {
      return false;
    }

    if (Math.abs(currentExtensionLength - this.m_extensionLengthSetpoint) > 2) {
      return false;
    }

    return true;
  }

  @Override
  public void periodic() {

    // Wrist
    if (getAbsolutePivotAngle() < Constants.Arm.kFrameAngle) {
      if (isInWristBounds(m_wristMotor.getEncoder().getPosition())
          || isInWristBounds(this.m_wristAngleSetpoint)) {
        setAbsoluteWristAngle(this.m_wristAngleSetpoint);
      } else if (m_wristMotor.getOutputCurrent() > 25) {
        setAbsoluteWristAngle(m_wristMotor.getEncoder().getPosition());
      } else {
        setAbsoluteWristAngle(m_wristMotor.getEncoder().getPosition());
      }
    }

    // Intake
    runIntake(m_intakeSpeedSetpoint);

    // Pivot
    if(!isArmSwitchActive() && previousLimitSwitchTrigger) {
      Constants.Arm.kAbsolutePivotOffset = 0;
      double difference = Constants.Arm.kMaxPivotAngle - getAbsolutePivotAngle() - 10;
      Constants.Arm.kAbsolutePivotOffset = difference;
    }

    previousLimitSwitchTrigger = isArmSwitchActive();

    if ((isInPivotBounds(getAbsolutePivotAngle()) && !isArmSwitchActive())
        || isInPivotBounds(this.m_pivotAngleSetpoint)) {
      setAbsolutePivotAngle(this.m_pivotAngleSetpoint);
    } else if (m_leftPivotMotor.getOutputCurrent() > Constants.Arm.kPivotAngleMaxAmperage) {
      m_leftPivotMotor.stopMotor();
    } else {
      setAbsolutePivotAngle(getAbsolutePivotAngle());
    }

    // Extension
    double limitedExtension = dynamicExtensionLimit(m_extensionLengthSetpoint);

    SmartDashboard.putNumber("Limited length", limitedExtension);

    if (isExtensionMinSwitchActive() && !m_extensionLimitSwitchReset) {
      // Rezero extension
      m_extensionLimitSwitchReset = true;
      m_extensionMotor.getEncoder().setPosition(0);
    } else {
      if ((!isExtensionMinSwitchActive() && !isExtensionMaxSwitchActive())
          || isInExtensionBounds(limitedExtension)) {
        setMotorExtensionLength(limitedExtension);
      } else {
        setMotorExtensionLength(getExtensionLength());
      }
    }

    // Track Values
    SmartDashboard.putNumber("Wrist Angle", getAbsoluteWristAngle());
    SmartDashboard.putNumber("Wrist Relative Angle", m_wristMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Pivot Angle", getAbsolutePivotAngle());
    SmartDashboard.putNumber("Extension Length", getExtensionLength());

    SmartDashboard.putNumber("WristSetpoint", getWristAngleSetpoint());
    SmartDashboard.putNumber("Pivot Setpoint", getPivotAngleSetpoint());
    SmartDashboard.putNumber("Extension Setpoint", getExtensionLengthSetpoint());
    SmartDashboard.putBoolean("Min Extension", isExtensionMinSwitchActive());
    SmartDashboard.putBoolean("Max Extension", isExtensionMaxSwitchActive());

    SmartDashboard.putBoolean("Arm Switch", isArmSwitchActive());
  }

  /**
   * Returns the object for the wrist motor.
   * 
   * @return the wrist motor.
   */
  public CougarSparkMax getWristMotor() {
    return m_wristMotor;
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
   * Returns the object for the intake motor.
   * 
   * @return The intake motor.
   */
  public CANSparkMax getIntakeMotor() {
    return m_intakeMotor;
  }

  /**
   * Returns the object for the extension motor.
   * 
   * @return the extension motor.
   */
  public CANSparkMax getExtensionMotor() {
    return m_extensionMotor;
  }

  /**
   * Returns the setpoint for the wrist.
   * 
   * @return the setpoint in degrees.
   */
  public double getWristAngleSetpoint() {
    return m_wristAngleSetpoint;
  }

  /**
   * Returns the setpoint for the pivot.
   * 
   * @return the setpoint in degrees.
   */
  public double getPivotAngleSetpoint() {
    return m_pivotAngleSetpoint;
  }

  /**
   * Returns the setpoint for the intake speed.
   * 
   * @return the speed of the intake between -1 and 1, inclusive.
   */
  public double getIntakeSpeedSetpoint() {
    return m_intakeSpeedSetpoint;
  }

  /**
   * Returns the setpoint for the extension of the arm.
   * 
   * @return the setpoint in inches.
   */
  public double getExtensionLengthSetpoint() {
    return m_extensionLengthSetpoint;
  }
}