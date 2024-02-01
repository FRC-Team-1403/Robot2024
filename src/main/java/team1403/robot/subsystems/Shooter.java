package team1403.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;

public class Shooter extends SubsystemBase {

  private CougarSparkMax m_shooterMotorTop;
  private CougarSparkMax m_shooterMotorBottom;
  //private WpiLimitSwitch m_intakeLimitSwitch;
  private double lastSpeed = 0;
  private DigitalInput m_shooterPhotogate;

  public Shooter(CougarLibInjectedParameters injectedParameters) {
    m_shooterMotorTop = CougarSparkMax.makeBrushless("Top Shooter Motor", Constants.CanBus.shooterMotorTop, SparkRelativeEncoder.Type.kHallSensor);
    m_shooterMotorBottom = CougarSparkMax.makeBrushless("Bottom Shooter Motor", Constants.CanBus.shooterMotorBottom, SparkRelativeEncoder.Type.kHallSensor);
    //m_intakeLimitSwitch = new WpiLimitSwitch("Intake Limit Switch", Constants.RioPorts.intakeLimitSwitchPort);
    m_shooterPhotogate = new DigitalInput(Constants.RioPorts.shooterPhotogate);
  }

  public boolean shooterPhotogate() {
    return m_shooterPhotogate.get();
  }
  public boolean shooterReady() {
    if (lastSpeed == Math.abs(m_shooterMotorTop.getEncoder().getVelocity()) && lastSpeed == Math.abs(m_shooterMotorBottom.getEncoder().getVelocity())) {
      return true;
    }
    else {
      return false;
    }  
  }

  public boolean speedIsEqual() {
    if (Math.abs(m_shooterMotorTop.getEncoder().getVelocity()) == Math.abs(m_shooterMotorBottom.getEncoder().getVelocity())) {
        return true;
    } else {
        return false;
    }
  }

  public void stop() {
    m_shooterMotorTop.setSpeed(0);
    m_shooterMotorBottom.setSpeed(0);
  }

  public void setShooterSpeed(double speed) {
    lastSpeed = speed;
    m_shooterMotorTop.set(-(speed));
    m_shooterMotorBottom.set(speed);
    //if there is an error when testing (note doesn't get shot out) try changing the direction of the motor
  
  }

  public void periodic() {
    Logger.recordOutput("Shooter Top Motor Temp", m_shooterMotorTop.getMotorTemperature());
    Logger.recordOutput("Shooter Bottom Motor Temp", m_shooterMotorBottom.getMotorTemperature());
    Logger.recordOutput("Shooter Top Motor RPM", m_shooterMotorTop.getVoltageCompensationNominalVoltage());
    Logger.recordOutput("Shooter Bottom Motor RPM", m_shooterMotorBottom.getVoltageCompensationNominalVoltage());
  }
}
