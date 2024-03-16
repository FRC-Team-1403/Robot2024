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

  private double m_speed;

  public HangerSubsystem() {
    m_leftServo = new Servo(Constants.RioPorts.kleftServoID);
    m_rightServo = new Servo(Constants.RioPorts.krightServoID);
    m_leftMotor = new CANSparkMax(Constants.CanBus.leftHangerMotorID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.CanBus.rightHangerMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setInverted(true);
    m_leftMotor.setInverted(false);
    // m_hangerLimtiSwitchLeftBottom = new
    // DigitalInput(Constants.RioPorts.kHangerLimitRightBottomID);
    // m_hangerLimitSwitchRightBottom = new
    // DigitalInput(Constants.RioPorts.kHangerLimitLeftBottomID);

    m_leftMotor.getEncoder().setPosition(0);
    m_rightMotor.getEncoder().setPosition(0);

    unlockHanger();
    m_speed = -0.5;
  }

  private void setHangerSpeed(double speed) {
    m_leftMotor.set(MathUtil.clamp(speed, -1, 1));
    m_rightMotor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void runHanger(double speed) {
    m_speed = speed;
  }

  public void stopHanger() {
    setHangerSpeed(0);
  }

  public void unlockHanger() {
    m_leftServo.setAngle(Constants.Hanger.kLeftUnlockAngle);
    m_rightServo.setAngle(Constants.Hanger.kRightUnlockAngle);
  }

  public void lockHanger() {
    m_leftServo.setAngle(Constants.Hanger.kLeftLockAngle);
    m_rightServo.setAngle(Constants.Hanger.kRightLockAngle);
  }
  // TODO Check if this is okay

  public boolean isAtTopLeft() {
   return m_leftMotor.getEncoder().getPosition() >= Constants.Hanger.kTopLeftLimit; 
  }

  public boolean isAtTopRight() {
   return m_rightMotor.getEncoder().getPosition() >= Constants.Hanger.kTopRightLimit; 
  } 

  public boolean isAtBottomLeft() {
    return m_leftMotor.getEncoder().getPosition() <= Constants.Hanger.kBottomLeftLimit;
  }

  public boolean isAtBottomRight() {
    return m_rightMotor.getEncoder().getPosition() <= Constants.Hanger.kBottomRightLimit;
  }

  public void periodic() {
    SmartDashboard.putNumber("Right Hanger Speed", m_rightMotor.get());
    SmartDashboard.putNumber("Left Hanger Speed", m_leftMotor.get());
    SmartDashboard.putNumber("Right Hanger Encoder", m_rightMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("Is at Top Right", isAtTopRight());
    SmartDashboard.putBoolean("Is at Bottom Left", isAtBottomLeft());
    SmartDashboard.putBoolean("Is at Top Left", isAtTopLeft());
    SmartDashboard.putBoolean("Is at Bottom Right", isAtBottomRight());
    
  
    if (isAtTopLeft()) {
      m_leftMotor.set(MathUtil.clamp(m_speed, -1, 0));
    } else if (isAtBottomLeft()) {
      m_leftMotor.set(MathUtil.clamp(m_speed, 0, 1));
    } else {
      m_leftMotor.set(m_speed);
    }
    if (isAtTopRight()) {
      m_rightMotor.set(MathUtil.clamp(m_speed, -1, 0));
    } else if (isAtBottomRight()) {
      m_rightMotor.set(MathUtil.clamp(m_speed, 0, 1));
    } else {
      m_rightMotor.set(m_speed);
    }
    SmartDashboard.putNumber("Left Motor Encoder", m_leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Motor Encoder", m_rightMotor.getEncoder().getPosition());

    // SmartDashboard.putNumber("Left Servo Angle", m_leftServo.getAngle());
    // SmartDashboard.putNumber("Right Servo Angle", m_rightServo.getAngle());
  }
}
