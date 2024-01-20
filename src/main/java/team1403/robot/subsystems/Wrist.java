package team1403.robot.subsystems;


import org.littletonrobotics.junction.Logger;


import com.revrobotics.SparkRelativeEncoder;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;


public class Wrist extends SubsystemBase {
   private CougarSparkMax m_wristMotor;
   private double lastSpeed = 0;


public Wrist() {
   m_wristMotor = CougarSparkMax.makeBrushless("Wrist Motor", Constants.CanBus.wristMotor, SparkRelativeEncoder.Type.kHallSensor);
 }


 public boolean wristReady() {
   if (lastSpeed == Math.abs(m_wristMotor.getEncoder().getVelocity())) {
     return true;
   }
   else {
     return false;
   } 
 }


 public void stop() {
   m_wristMotor.setSpeed(0);
 }


 public void setWristSpeed(double speed) {
   lastSpeed = speed;
   m_wristMotor.set(speed); 
 }


public void periodic() {
   Logger.recordOutput("Wrist Temp", m_wristMotor.getMotorTemperature());
 }
}
