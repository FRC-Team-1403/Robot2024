package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;


public class Wrist extends SubsystemBase {
    private CougarSparkMax m_wristMotor;
    private final PIDController m_wristPid;
    private final ArmFeedforward m_wristFeedforward;
    private double m_wristAngle;
    private double lastSpeed = 0;

  public Wrist() {
    m_wristMotor = CougarSparkMax.makeBrushless("Wrist Motor", Constants.CanBus.wristMotor, SparkRelativeEncoder.Type.kHallSensor);
    m_wristPid = new PIDController(Constants.Wrist.kPWrist, Constants.Wrist.KIWrist, Constants.Wrist.KDWrist);
    m_wristFeedforward = new ArmFeedforward(Constants.Wrist.kSWrist, Constants.Wrist.kGWrist, Constants.Wrist.kVWrist, Constants.Wrist.kAWrist);

  }
  public double getWristAngle() {
    return m_wristAngle;
    
  }

  public void setWristAngle(){
    m_wristMotor.set(m_wristPid.calculate(m_wristMotor.getEncoder().getPosition(), 10));


  }



  public boolean wristReady() {
    if (lastSpeed == Math.abs(m_wristMotor.getEncoder().getVelocity())) {
      return true;
    }
    else {
      return false;
    }  
  }

  // public void stop() {
  //   m_wristMotor.setSpeed(0);
  // }

  public void setWristSpeed(double speed) {
    lastSpeed = speed;
    m_wristMotor.set(speed);  
  }

  public void softTopLimitWrist(){
    m_wristFeedforward.calculate(Constants.Wrist.topLimit, Constants.Wrist.feedforwardVelocity, Constants.Wrist.feedforwardAcc);
  }

  public void softBottomLimitWrist(){
    m_wristFeedforward.calculate(Constants.Wrist.bottomLimit, Constants.Wrist.feedforwardVelocity, Constants.Wrist.feedforwardAcc);
  }

public void periodic() {
  m_wristAngle = m_wristMotor.getEncoder().getPosition();

   Logger.recordOutput("Wrist Temp", m_wristMotor.getMotorTemperature());
   Logger.recordOutput("Wrist Motor RPM", m_wristMotor.getVoltageCompensationNominalVoltage());
   Logger.recordOutput("Wrist Angle", getWristAngle());  }
}
