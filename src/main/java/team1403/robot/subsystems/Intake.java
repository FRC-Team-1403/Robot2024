package team1403.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
//import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;

public class Intake extends SubsystemBase {
  private CougarSparkMax m_intakeMotorTop;
  private CougarSparkMax m_intakeMotorBottom;
  //private WpiLimitSwitch m_intakeLimitSwitch;
  private DigitalInput m_intakePhotogate1;
  private DigitalInput m_intakePhotogate2;

  public Intake(CougarLibInjectedParameters injectedParameters) {
    m_intakeMotorTop = CougarSparkMax.makeBrushless("Top Intake Motor", Constants.CanBus.intakeMotorTop, SparkRelativeEncoder.Type.kHallSensor);
    m_intakeMotorBottom = CougarSparkMax.makeBrushless("Bottom Intake Motor", Constants.CanBus.intakeMotorBottom, SparkRelativeEncoder.Type.kHallSensor);
    //m_intakeLimitSwitch = new WpiLimitSwitch("Intake Limit Switch", Constants.RioPorts.intakeLimitSwitchPort);
    m_intakePhotogate1 = new DigitalInput(Constants.RioPorts.intakePhotogate1);
    m_intakePhotogate2 = new DigitalInput(Constants.RioPorts.kIntakePhotogae2);
  }

  public boolean intakePhotogate1() {
    return m_intakePhotogate1.get();
  }

  public boolean intakePhotogate2(){
    return m_intakePhotogate2.get();
  }

  private CougarSparkMax m_motorTop;
  private CougarSparkMax m_motorBottom;
  private double lastSpeed = 0;

  public Intake() {
    m_motorTop = CougarSparkMax.makeBrushless(
      "Top Intake Motor", Constants.CanBus.intakeMotorTop, SparkRelativeEncoder.Type.kHallSensor);
    m_motorBottom = CougarSparkMax.makeBrushless(
      "Bottom Intake Motor", Constants.CanBus.intakeMotorBottom, SparkRelativeEncoder.Type.kHallSensor);
  }

  public boolean intakeReady() {
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
    m_intakeMotorTop.setSpeed(0);
    m_intakeMotorBottom.setSpeed(0);
  }

  public void setIntakeSpeed(double speed) {
     //if (m_intakeLimitSwitch.get()) {
      //stop();
      //return;
    //}
    lastSpeed = speed;
    m_intakeMotorTop.set(speed);
    m_intakeMotorBottom.set(-(speed));
    //if there is an error when testing (note doesn't get taken in) try changing the direction of the motor
  
  }

  public void periodic() {
    Logger.recordOutput("Intake Top Motor Temp", m_intakeMotorTop.getMotorTemperature());
    Logger.recordOutput("Intake Bottom Motor Temp", m_intakeMotorBottom.getMotorTemperature());
    Logger.recordOutput("Intake Top Motor RPM", m_intakeMotorTop.getVoltageCompensationNominalVoltage());
    Logger.recordOutput("Intake Bottom Motor RPM", m_intakeMotorBottom.getVoltageCompensationNominalVoltage());
  }
}
