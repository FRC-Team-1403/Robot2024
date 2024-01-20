package team1403.robot.subsystems;


import org.littletonrobotics.junction.Logger;


import com.revrobotics.SparkRelativeEncoder;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;


public class Shooter extends SubsystemBase {


 private CougarSparkMax m_motorTop;
 private CougarSparkMax m_motorBottom;
 private double lastSpeed = 0;


 public Shooter() {
   m_motorTop = CougarSparkMax.makeBrushless(
     "Top Shooter Motor", Constants.CanBus.shooterMotorTop, SparkRelativeEncoder.Type.kHallSensor);
   m_motorBottom = CougarSparkMax.makeBrushless(
     "Bottom Shooter Motor", Constants.CanBus.shooterMotorBottom, SparkRelativeEncoder.Type.kHallSensor);
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


 public void setShooterSpeed(double speed) {
   lastSpeed = speed;
   m_motorTop.set(speed);
   m_motorBottom.set(-(speed));
   //if there is an error when testing (note doesn't shoot) try changing the direction of the motor
 }


 public void periodic() {
   Logger.recordOutput("Shooter Top Motor Temp", m_motorTop.getMotorTemperature());
   Logger.recordOutput("Shooter Bottom Motor Temp", m_motorBottom.getMotorTemperature());
   Logger.recordOutput("Shooter Top Motor RPM", m_motorTop.getVoltageCompensationNominalVoltage());
   Logger.recordOutput("Shooter Bottom Motor RPM", m_motorBottom.getVoltageCompensationNominalVoltage());
 }
}
