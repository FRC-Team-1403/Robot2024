package team1403.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;


public class Wrist extends SubsystemBase {
    private CougarSparkMax m_wristMotor;
    private final PIDController m_wristPid;
    //private final ArmFeedforward m_wristFeedforward;
    private DutyCycleEncoder m_wristAbsoluteEncoder;
    private double m_wristAngle;
    private ArmFeedforward m_feedforward;
    private ArmFeedforward m_feedforward2;

    private Mechanism2d m_mechanism;
    private MechanismRoot2d m_mechanismRoot;
    private MechanismLigament2d m_armMech;
    private MechanismLigament2d m_wristMech;

    private double m_wristAngleSetpoint;
    private double m_wristMotorSpeed;

    private ArmSubsystem m_arm;

  public Wrist(ArmSubsystem arm) {
    // m_feedforward = new ArmFeedforward(0, 0.08, 0.0001);
    // m_feedforward2 = new ArmFeedforward(0, 0.04, 0.0001);
    m_wristMotor = CougarSparkMax.makeBrushless("Wrist Motor", Constants.CanBus.wristMotorID, SparkRelativeEncoder.Type.kHallSensor);
    m_wristPid = new PIDController(Constants.Wrist.KPWrist, Constants.Wrist.KIWrist, Constants.Wrist.KDWrist);
    //m_wristFeedforward = new ArmFeedforward(Constants.Wrist.kSWrist, Constants.Wrist.kGWrist, Constants.Wrist.kVWrist, Constants.Wrist.kAWrist);
    m_wristAbsoluteEncoder = new DutyCycleEncoder(Constants.RioPorts.kwristAbsoluteEncoder);

    m_wristMotor.setIdleMode(IdleMode.kBrake);
    m_arm = arm;

    m_wristAngleSetpoint = Constants.Wrist.kIntakeSetpoint;
    m_wristMotorSpeed = 0;

    m_mechanism = new Mechanism2d(3, 3);
    m_mechanismRoot = m_mechanism.getRoot("A-Frame", 1, 1);
    m_armMech = m_mechanismRoot.append(new MechanismLigament2d("Arm", 1, m_arm.getPivotAngle() - 106.4));
    m_wristMech = m_armMech.append(new MechanismLigament2d("Wrist", 0.3, getWristAngle()));

    SmartDashboard.putData("Arm Mechanism", m_mechanism);

    // SmartDashboard.putNumber("Wrist P", m_wristPid.getP());
  }
  
  public double getWristAngle() {
    return m_wristAngle;
  }

  public void setWristAngle(double wristAngle) {
    m_wristAngleSetpoint = limitAngle(wristAngle);
  }
  
  public void increaseWristAngle(double wristAngleIncrement) {
    m_wristAngleSetpoint += wristAngleIncrement;
    m_wristAngleSetpoint = limitAngle(m_wristAngleSetpoint);
  }
  
  //Bound while shooting
  public boolean isInBounds(double angle) {
    return (angle <= Constants.Wrist.kTopLimit && angle >= Constants.Wrist.kBottomLimit);
  }

  // public boolean isInTuckBound(double angle) {
  //   return (angle <= Constants.Wrist.kTopIntakeLimit && angle >= Constants.Wrist.kBottomIntakeLimit);
  // }

  public double limitAngle(double angle) {
    return MathUtil.clamp(angle, Constants.Wrist.kBottomLimit, Constants.Wrist.kTopLimit);
  }

  // public double limitTuckAngle(double angle) {
  //   return MathUtil.clamp(m_wristAngle, Constants.Wrist.kBottomIntakeLimit, Constants.Wrist.kTopIntakeLimit);
  // }

  public boolean isAtSetpoint() {
    return Math.abs(getWristAngle() - m_wristAngleSetpoint) <= 5.0;
  }

  public void setWristSpeed(double speed) {
    //double feedforward = m_feedforward.calculate(Units.degreesToRadians(getWristAngle()), 0);
    //if(getWristAngle() > Constants.Arm.kArmAngle) 
    //feedforward = m_feedforward2.calculate(Units.degreesToRadians(getWristAngle()), 0);
    if(isOverUpperBound()) {
      m_wristMotorSpeed = MathUtil.clamp(speed, -0.1, 0);
    }
    else if(isUnderLowerBound()) { 
      m_wristMotorSpeed = MathUtil.clamp(speed, 0, 0.1);
    }
    else {
      m_wristMotorSpeed = MathUtil.clamp(speed,-0.75,0.75);
    }
      
  }

  private boolean isOverUpperBound(){
    return (getWristAngle() >= Constants.Wrist.kTopLimit);
  }

  private boolean isUnderLowerBound(){
    return (getWristAngle() <= Constants.Wrist.kBottomLimit);
  }

  @Override
  public void periodic() {
    m_wristAngle = m_wristAbsoluteEncoder.getAbsolutePosition() * 360.0;

    // tune PID with shuffleboard
    // m_wristPid.setP(SmartDashboard.getNumber("Wrist P", m_wristPid.getP()));

   if(isInBounds(m_wristAngleSetpoint) && m_arm.getPivotAngle() > Constants.Arm.kMinPivotAngle)
    setWristSpeed(m_wristPid.calculate(m_wristAngle, m_wristAngleSetpoint));

    m_wristMotor.set(m_wristMotorSpeed);
    //m_wristPid.setP(Constants.Wrist.KPWrist);

    m_armMech.setAngle(m_arm.getPivotAngle() - 106.4);
    m_wristMech.setAngle(-getWristAngle() + 90);

   SmartDashboard.putBoolean("Wrist is at setpoint", isAtSetpoint());
   Logger.recordOutput("Wrist Voltage", m_wristMotor.getBusVoltage());
   Logger.recordOutput("Wrist Current", m_wristMotor.getOutputCurrent());
   Logger.recordOutput("wrist rpm", m_wristMotor.getEmbeddedEncoder().getVelocityValue());
  

   Logger.recordOutput("Wrist Temp", m_wristMotor.getMotorTemperature());
   Logger.recordOutput("Wrist Motor RPM", m_wristMotor.getEmbeddedEncoder().getVelocityValue());
   Logger.recordOutput("Wrist Speed", m_wristMotor.get());
   Logger.recordOutput("Wrist Angle", getWristAngle());
   Logger.recordOutput("Wrist Setpoint", m_wristAngleSetpoint);
  
  }
}
