package team1403.robot.subsystems;


import org.littletonrobotics.junction.Logger;


import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;


public class Intake extends SubsystemBase {


 private CougarSparkMax m_motorTop;
 private CougarSparkMax m_motorBottom;
 private DigitalInput m_photogate;
 private double lastSpeed = 0;


 public Intake() {
   m_motorTop = CougarSparkMax.makeBrushless(
     "Top Intake Motor", Constants.CanBus.intakeMotorTop, SparkRelativeEncoder.Type.kHallSensor);
   m_motorBottom = CougarSparkMax.makeBrushless(
     "Bottom Intake Motor", Constants.CanBus.intakeMotorBottom, SparkRelativeEncoder.Type.kHallSensor);
   m_photogate = new DigitalInput(Constants.RioPorts.intakePhotogate);
 }

 public boolean intakePhotogate() {
  return m_photogate.get();
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
   m_motorTop.setSpeed(0);
   m_motorBottom.setSpeed(0);
 }


 public void setIntakeSpeed(double speed) {
   lastSpeed = speed;
   m_motorTop.set(speed);
   m_motorBottom.set(-(speed));
   //if there is an error when testing (note doesn't get taken in) try changing the direction of the motor
 }


 public void periodic() {
   Logger.recordOutput("Intake Top Motor Temp", m_motorTop.getMotorTemperature());
   Logger.recordOutput("Intake Bottom Motor Temp", m_motorBottom.getMotorTemperature());
   Logger.recordOutput("Intake Top Motor RPM", m_motorTop.getVoltageCompensationNominalVoltage());
   Logger.recordOutput("Intake Bottom Motor RPM", m_motorBottom.getVoltageCompensationNominalVoltage());
 }
}
