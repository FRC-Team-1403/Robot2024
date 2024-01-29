package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;


public class Wrist extends SubsystemBase {
   private CougarSparkMax m_wristMotor;
   private final DutyCycleEncoder m_wristBoreEncoder;

public Wrist() {
   m_wristMotor = CougarSparkMax.makeBrushless("Wrist Motor", Constants.CanBus.wristMotor, SparkRelativeEncoder.Type.kHallSensor);
   m_wristBoreEncoder = new DutyCycleEncoder(Constants.RioPorts.kArmBoreEncoder);
 }

 public void stop() {
   m_wristMotor.setSpeed(0);
 }

 public void setWristSpeed(double speed) {
  if (getWristAngle() > 90.0 || getWristAngle() < 0) {     //update limit if needed
    stop();
    return;
  }  
  m_wristMotor.set(speed); 
 }

 public double getWristAngle() {
  return m_wristBoreEncoder.get();
 }

public void periodic() {
   Logger.recordOutput("Wrist Temp", m_wristMotor.getMotorTemperature());
   Logger.recordOutput("Wrist Motor RPM", m_wristMotor.getVoltageCompensationNominalVoltage());
   Logger.recordOutput("Wrist Angle", getWristAngle());
 }
}
