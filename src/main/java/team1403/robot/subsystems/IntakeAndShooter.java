package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
//import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;

public class IntakeAndShooter extends SubsystemBase {

  private CougarSparkMax m_motorTop;
  private CougarSparkMax m_motorBottom;
  //private WpiLimitSwitch m_intakeLimitSwitch;
  private double lastSpeed = 0;
  private DigitalInput m_intakePhotogate;
  private DigitalInput m_shooterPhotogate;

  public IntakeAndShooter(CougarLibInjectedParameters injectedParameters) {
    m_motorTop = CougarSparkMax.makeBrushless(
      "Top Intake/Shooter Motor", Constants.CanBus.intakeAndShooterMotorTop, SparkRelativeEncoder.Type.kHallSensor);
    m_motorBottom = CougarSparkMax.makeBrushless(
      "Bottom Intake/Shooter Motor", Constants.CanBus.intakeAndShooterMotorBottom, SparkRelativeEncoder.Type.kHallSensor);
    //m_intakeLimitSwitch = new WpiLimitSwitch(
      //"Intake Limit Switch", Constants.RioPorts.intakeLimitSwitchPort);
  }

  public boolean intakePhotogate() {
    return m_intakePhotogate.get();
  }

  public boolean shooterPhotogate() {
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
    m_motorTop.setSpeed(0);
    m_motorBottom.setSpeed(0);
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
    Logger.recordOutput("Intake/Shooter Top Motor Temp", m_motorTop.getMotorTemperature());
    Logger.recordOutput("Intake/Shooter Bottom Motor Temp", m_motorBottom.getMotorTemperature());
    Logger.recordOutput("Intake/Shooter Top Motor RPM", m_motorTop.getVoltageCompensationNominalVoltage());
    Logger.recordOutput("Intake/Shooter Bottom Motor RPM", m_motorBottom.getVoltageCompensationNominalVoltage());
  }
}
