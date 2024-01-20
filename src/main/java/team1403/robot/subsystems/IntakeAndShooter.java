package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
//import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;
import team1403.robot.Constants.CanBus;

public class IntakeAndShooter extends SubsystemBase {

  private CANSparkMax m_motorTop;
  private CANSparkMax m_motorBottom;
  //private WpiLimitSwitch m_intakeLimitSwitch;
  private double lastSpeed = 0;
  private DigitalInput m_intakePhotogate;
  private DigitalInput m_shooterPhotogate;

  public IntakeAndShooter() {
    m_motorTop = new CANSparkMax(Constants.CanBus.intakeAndShooterMotorTop, MotorType.kBrushless);
    m_motorBottom = new CANSparkMax(Constants.CanBus.intakeAndShooterMotorBottom, MotorType.kBrushless);
    //m_intakeLimitSwitch = new WpiLimitSwitch("Intake Limit Switch", Constants.RioPorts.intakeLimitSwitchPort);
  }

  public boolean intakePhotogateValue() {
    return m_intakePhotogate.get();
  }

  public boolean shooterPhotogateValue() {
    return m_shooterPhotogate.get();
  }

  public boolean intakeReady() {
    if (lastSpeed == Math.abs(m_motorTop.getEncoder().getVelocity()) && lastSpeed == Math.abs(m_motorBottom.getEncoder().getVelocity())) {
      return true;
    }
    else {
      return false;
    }  
  }

  public boolean shooterReady() {
    if (lastSpeed == Math.abs(m_motorTop.getEncoder().getVelocity()) && lastSpeed == Math.abs(m_motorBottom.getEncoder().getVelocity())) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean speedIsEqual() {
    if (Math.abs(m_motorTop.getEncoder().getVelocity()) == Math.abs(m_motorBottom.getEncoder().getVelocity())) {
        return true;
    } else {
        return false;
    }
  }

  public void stop() {
    m_motorTop.set(0);
    m_motorBottom.set(0);
  }



  public void setIntakeSpeed(double speed) {
     //if (m_intakeLimitSwitch.get()) {
      //stop();
      //return;
    //}
    lastSpeed = speed;
    m_motorTop.set(speed);
    m_motorBottom.set(-(speed));
    //if there is an error when testing (note doesn't get taken in) try changing the direction of the motor
  
  }

  public void setShooterSpeed(double speed) {
    lastSpeed = speed;
    m_motorTop.set(-(speed));
    m_motorBottom.set(speed);
    //if there is an error when testing (note doesn't get shot out) try changing the direction of the motor
  
  }

  public void periodic() {
    Logger.recordOutput("Intake/Shooter Temp", m_motorTop.getMotorTemperature());
    Logger.recordOutput("Intake/Shooter Temp", m_motorBottom.getMotorTemperature());
  }
}
