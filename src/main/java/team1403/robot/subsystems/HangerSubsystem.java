package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class HangerSubsystem extends SubsystemBase {
  private DigitalInput m_hangerLimtiSwitchLeftBottom;
  private DigitalInput m_hangerLimitSwitchRightBottom;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private Servo m_leftServo;
  private Servo m_rightServo;


  public HangerSubsystem() {
    m_leftServo = new Servo(Constants.RioPorts.kleftServoID);
    m_rightServo = new Servo(Constants.RioPorts.krightServoID);
    
    //m_leftMotor = new CANSparkMax(Constants.CanBus.leftHangerMotorID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.CanBus.rightHangerMotorID, MotorType.kBrushless);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    //m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setInverted(true);

    // m_hangerLimtiSwitchLeftBottom = new DigitalInput(Constants.RioPorts.kHangerLimitRightBottomID);
    // m_hangerLimitSwitchRightBottom = new DigitalInput(Constants.RioPorts.kHangerLimitLeftBottomID);

    //m_rightMotor.follow(m_leftMotor);
    
    //m_leftMotor.getEncoder().setPosition(0);
    m_rightMotor.getEncoder().setPosition(0);
  }

  private void setHangerSpeed(double speed) {
    // m_leftMotor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void runHanger(double speed) {
    if (isAtTop()) {
      setHangerSpeed(MathUtil.clamp(speed, -1, 0));
    } else if (isAtBottom()) {
      setHangerSpeed(MathUtil.clamp(speed, 0, 1));
    } else {
      setHangerSpeed(speed);
    }
  }

  public void stopHanger() {
    setHangerSpeed(0);
  }

  public void setServoAngle(double angle) {
    m_leftServo.setAngle(angle);
    m_rightServo.setAngle(angle);
  }

  public void unlockHanger() {
    setServoAngle(Constants.Hanger.kUnlockAngle);
  }

  public void lockHanger() {
    setServoAngle(Constants.Hanger.kLockAngle);
  }

  public boolean isAtTop() {
   return m_leftMotor.getEncoder().getPosition() >= Constants.Hanger.kTopLimit; 
  }

  public boolean isAtBottom() {
    return m_leftMotor.getEncoder().getPosition() <= Constants.Hanger.kBottomLimit;
  }

  public void periodic() {
    SmartDashboard.putNumber("Right Hanger Speed", m_rightMotor.get());
    SmartDashboard.putNumber("Right Hanger Encoder", m_rightMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Left Servo Angle", m_leftServo.getAngle());
    SmartDashboard.putNumber("Right Servo Angle", m_rightServo.getAngle());
  }
}
