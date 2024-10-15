package team1403.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.util.CougarLogged;
import team1403.robot.Constants;

public class ArmWristSubsystem extends SubsystemBase implements CougarLogged {
  // lead motor
  private final CANSparkMax m_leftMotor;
  // following motor
  private final CANSparkMax m_rightMotor;
  private final CougarSparkMax m_wristMotor;
  private final DutyCycleEncoder m_armEncoder;
  private final DutyCycleEncoder m_wristEncoder;
  private final ProfiledPIDController m_armPid;
  private final ProfiledPIDController m_wristPid;
  private final ArmFeedforward m_feedforward;

  // Setpoints
  private double m_pivotAngleSetpoint;
  private double m_wristAngleSetpoint;


  private Mechanism2d m_mechanism;
  private MechanismRoot2d m_mechanismRoot;
  private MechanismLigament2d m_armMech;
  private MechanismLigament2d m_wristMech;

  public ArmWristSubsystem() {
    m_feedforward = new ArmFeedforward(0, Constants.Arm.kFeedforwardG, Constants.Arm.kFeedforwardV);
    m_leftMotor = new CANSparkMax(Constants.CanBus.leftPivotMotorID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.CanBus.rightPivotMotorID, MotorType.kBrushless);
    m_wristMotor = CougarSparkMax.makeBrushless("Wrist Motor", Constants.CanBus.wristMotorID, SparkRelativeEncoder.Type.kHallSensor);
    m_armEncoder = new DutyCycleEncoder(Constants.RioPorts.kArmAbsoluteEncoder);
    m_wristEncoder = new DutyCycleEncoder(Constants.RioPorts.kwristAbsoluteEncoder);

    configPivotMotors();
    configWristMotor();

    m_armPid = new ProfiledPIDController(Constants.Arm.KPArmPivot, Constants.Arm.KIArmPivot, Constants.Arm.KDArmPivot, new TrapezoidProfile.Constraints(370, 570));
    m_wristPid = new ProfiledPIDController(Constants.Wrist.KPWrist, Constants.Wrist.KIWrist, Constants.Wrist.KDWrist, new TrapezoidProfile.Constraints(200, 600));
    m_armPid.reset(getPivotAngle(), 0);
    m_wristPid.reset(getWristAngle(), 0);

    m_mechanism = new Mechanism2d(3, 3);
    m_mechanismRoot = m_mechanism.getRoot("A-Frame", 1, 1);
    m_armMech = m_mechanismRoot.append(new MechanismLigament2d("Arm", 1, getPivotAngle() - 106.4));
    m_wristMech = m_armMech.append(new MechanismLigament2d("Wrist", 0.3, getWristAngle()));

    if(Constants.DEBUG_MODE) {
      Constants.kDebugTab.add("Arm Mechanism", m_mechanism);
      Constants.kDebugTab.addBoolean("Wrist is at Setpoint", () -> isWristAtSetpoint());
      Constants.kDebugTab.addBoolean("Arm IsAtSetpoint", () -> isArmAtSetpoint());
      Constants.kDebugTab.add("Arm PIDController", m_armPid);
      Constants.kDebugTab.add("Wrist PIDController", m_wristPid);
    }

    m_pivotAngleSetpoint = getPivotAngle();
    m_wristAngleSetpoint = Constants.Wrist.kIntakeSetpoint;
  }

  private void configPivotMotors() {
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.enableVoltageCompensation(Constants.Arm.kPivotMotorVoltageLimit);
    m_leftMotor.setSmartCurrentLimit(Constants.Arm.kPivotMotorCurrentLimit);
    m_rightMotor.setSmartCurrentLimit(Constants.Arm.kPivotMotorCurrentLimit);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.enableVoltageCompensation(Constants.Arm.kPivotMotorVoltageLimit);
    m_rightMotor.follow(m_leftMotor, true);
  }

  private void configWristMotor() {
    m_wristMotor.setIdleMode(IdleMode.kBrake);
  }

  public double getPivotAngle() {
    return m_armEncoder.getAbsolutePosition() * 360;
  }

  public double getWristAngle() {
    return m_wristEncoder.getAbsolutePosition() * 360;
  }

  public boolean isArmAtSetpoint() {
    return Math.abs(getPivotAngle() - m_pivotAngleSetpoint) <= 5.0;
  }

  public boolean isWristAtSetpoint() {
    return Math.abs(getWristAngle() - m_wristAngleSetpoint) <= 5.0;
  }

  public void setArmSetpoint(double angle) {
    m_pivotAngleSetpoint = MathUtil.clamp(angle, Constants.Arm.kMinPivotAngle, Constants.Arm.kMaxPivotAngle);
  }

  public void setWristSetpoint(double angle) {
    m_wristAngleSetpoint = MathUtil.clamp(angle, Constants.Wrist.kWristLowerLimit, Constants.Wrist.kWristUpperLimit);
  }

  public void applySetpoint(SonicBlasterSetpoint setpoint) {
    setWristSetpoint(setpoint.getWristAngle());
    setArmSetpoint(setpoint.getArmAngle());
  }

  private boolean isWristInSafeBounds() {
    return getWristAngle() > 130 && getWristAngle() < 145;
  }

  private double calcPivotSpeed() {
    double setpoint = m_pivotAngleSetpoint;

    if(setpoint < 110 && !isWristInSafeBounds()) setpoint = MathUtil.clamp(setpoint, 110, Constants.Arm.kMaxPivotAngle);

    double speed = m_armPid.calculate(getPivotAngle(), setpoint);

    if(getPivotAngle() > Constants.Arm.kMaxPivotAngle)
        speed = MathUtil.clamp(speed, -0.1, 0);
    else if(getPivotAngle() < Constants.Arm.kMinPivotAngle)
        speed = MathUtil.clamp(speed, 0, 0.1);

    return speed + m_feedforward.calculate(Units.degreesToRadians(getPivotAngle() - 106.4), 0);
  }

  private double calcWristSpeed() {
    double setpoint = m_wristAngleSetpoint;

    if (getPivotAngle() < 110) setpoint = MathUtil.clamp(setpoint, Constants.Wrist.kIntakeSetpoint, 140);

    double speed = m_wristPid.calculate(getWristAngle(), setpoint);

    if(getWristAngle() < Constants.Wrist.kWristLowerLimit)
        speed = MathUtil.clamp(speed, 0, 0.1);
    else if(getWristAngle() > Constants.Wrist.kWristUpperLimit)
        speed = MathUtil.clamp(speed, -0.1, 0);

    return speed;
  }

  public double getPivotSetpoint() {
    return m_pivotAngleSetpoint;
  }

  @Override
  public void periodic() {

    m_wristMech.setAngle(-getWristAngle() + 90);
    m_armMech.setAngle(getPivotAngle() - 106.4);

    m_leftMotor.set(calcPivotSpeed());
    m_wristMotor.set(calcWristSpeed());


    log("Pivot/Angle", getPivotAngle());
    log("Pivot/Setpoint", m_pivotAngleSetpoint);
    log("Pivot/Voltage", m_leftMotor.getBusVoltage());
    log("Pivot/Speed", m_leftMotor.get());
    log("Wrist/Voltage", m_wristMotor.getBusVoltage());
    log("Wrist/Current", m_wristMotor.getOutputCurrent());
    log("Wrist/Temp", m_wristMotor.getMotorTemperature());
    log("Wrist/Motor RPM", m_wristMotor.getEmbeddedEncoder().getVelocityValue());
    log("Wrist/Speed", m_wristMotor.get());
    log("Wrist/Angle", getWristAngle());
    log("Wrist/Setpoint", m_wristAngleSetpoint);
  }
    
}
