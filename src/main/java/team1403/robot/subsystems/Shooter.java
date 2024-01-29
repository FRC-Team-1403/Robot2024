package team1403.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import team1403.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;


public class Shooter extends SubsystemBase {
 private DigitalInput m_shooterPhotoswitch;
 private double lastSpeed = 0;
 private TalonFX m_falconTop;
 private TalonFX m_falconBottom;

 public Shooter() {
  m_falconTop = new TalonFX(Constants.CanBus.shooterMotorTop);
  m_falconBottom = new TalonFX(Constants.CanBus.shooterMotorBottom);
  m_shooterPhotoswitch = new DigitalInput(Constants.RioPorts.shooterPhotoswitch);
 }

public boolean shooterPhotoswitch() {
  return m_shooterPhotoswitch.get();
}

 public boolean shooterReady() {
   if (lastSpeed == Math.abs(m_falconTop.get()) && lastSpeed == Math.abs(m_falconBottom.get())) {
     return true;
   }
   else {
     return false;
   } 
 }


 public boolean speedIsEqual() {
   if (Math.abs(m_falconTop.get()) == Math.abs(m_falconBottom.get())) {
       return true;
   } else {
       return false;
   }
 }


 public void stop() {
   m_falconTop.set(0);
   m_falconBottom.set(0);
 }


 public void setShooterSpeed(double speed) {
   lastSpeed = speed;
   m_falconTop.set(speed);
   m_falconBottom.set(-(speed));
   //if there is an error when testing (note doesn't shoot) try changing the direction of the motor
 }


 public void periodic() {
  //  Logger.recordOutput("Shooter Top Motor Temp", m_falconTop.getMotorTemperature());
  //  Logger.recordOutput("Shooter Bottom Motor Temp", m_falconBottom.getMotorTemperature());
   Logger.recordOutput("Shooter Top Motor RPM", m_falconTop.get());
   Logger.recordOutput("Shooter Bottom Motor RPM", m_falconBottom.get());
 }
}
