package team1403.robot.subsystems;


import org.littletonrobotics.junction.Logger;


import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;


public class Intake extends SubsystemBase {


 private CougarSparkMax m_intakeMotor;
 //private CougarSparkMax m_motorBottom;
 private DigitalInput m_intakePhotogate;
 private PIDController m_pidController;
 private double lastSpeed = 0;


 public Intake() {
   m_intakeMotor = CougarSparkMax.makeBrushless(
     "Top Intake Motor", Constants.CanBus.intakeMotor, SparkRelativeEncoder.Type.kHallSensor);
  //  m_motorBottom = CougarSparkMax.makeBrushless(
  //    "Bottom Intake Motor", Constants.CanBus.intakeMotorBottom, SparkRelativeEncoder.Type.kHallSensor);
   m_intakePhotogate = new DigitalInput(Constants.RioPorts.intakePhotogate);

   m_pidController = new PIDController(0.002, 0, 0);
 }

 public boolean intakePhotogate() {
  return m_intakePhotogate.get();
}

 public boolean intakeReady() {
   //if (lastSpeed == Math.abs(m_intakeMotor.getEncoder().getVelocity()) && lastSpeed == Math.abs(m_motorBottom.getEncoder().getVelocity())) {
   if (lastSpeed == Math.abs(m_intakeMotor.getEncoder().getVelocity())) { 
    return true;
   }
   else {
     return false;
   } 
 }


//  public boolean speedIsEqual() {
//    if (Math.abs(m_intakeMotor.getEncoder().getVelocity()) == Math.abs(m_motorBottom.getEncoder().getVelocity())) {
//        return true;
//    } else {
//        return false;
//    }
//  }


 public void stop() {
   m_intakeMotor.set(0);
   //m_motorBottom.setSpeed(0);
 }


 public void setIntakeSpeed(double speed) {
   lastSpeed = speed;
   m_intakeMotor.set(speed);
   //if there is an error when testing (note doesn't get taken in) try changing the direction of the motor
 }

 public void setIntakeRPM(double rpm)
 {
    double speed = m_pidController.calculate(m_intakeMotor.getEncoder().getVelocity(), rpm);
    setIntakeSpeed(lastSpeed + speed);
 }


 public void periodic() {
   Logger.recordOutput("Intake Motor Temp", m_intakeMotor.getMotorTemperature());
   //Logger.recordOutput("Intake Bottom Motor Temp", m_motorBottom.getMotorTemperature());
   Logger.recordOutput("Intake Motor RPM", m_intakeMotor.getEncoder().getVelocity());
   //Logger.recordOutput("Intake Bottom Motor RPM", m_motorBottom.getVoltageCompensationNominalVoltage());
 }
}
