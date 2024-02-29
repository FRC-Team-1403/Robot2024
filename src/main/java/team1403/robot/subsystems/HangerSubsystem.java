package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class HangerSubsystem extends SubsystemBase {
  private DigitalInput m_hangerLimtiSwitchLeftBottom;
  private DigitalInput m_hangerLimitSwitchRightBottom;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private Servo m_leftSwervo;
  private Servo m_rightServo;


  public HangerSubsystem() {
    m_leftSwervo = new Servo(Constants.RioPorts.kleftServoID);
    m_rightServo = new Servo(Constants.RioPorts.krightServoID);
    
    m_leftMotor = new CANSparkMax(Constants.RioPorts.kleftHangerMotorID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.RioPorts.krightHangerMotorID, MotorType.kBrushless);

    m_hangerLimtiSwitchLeftBottom = new DigitalInput(Constants.RioPorts.kHangerLimitRightBottomID);
    m_hangerLimitSwitchRightBottom = new DigitalInput(Constants.RioPorts.kHangerLimitLeftBottomID);

    m_rightMotor.follow(m_leftMotor);
  }

  private void setHangerSpeed(double speed) {
    m_leftMotor.set(MathUtil.clamp(speed, -1, 1));
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

  private void setServoAngle(double angle) {
    m_leftSwervo.setAngle(angle);
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
    return m_hangerLimtiSwitchLeftBottom.get() && m_hangerLimitSwitchRightBottom.get();
  }

  public void periodic() {
    if(isAtBottom())
    {
      m_leftMotor.getEncoder().setPosition(0);
      m_rightMotor.getEncoder().setPosition(0);
    }
  }
}
