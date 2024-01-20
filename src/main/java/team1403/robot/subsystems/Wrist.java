package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class Wrist extends SubsystemBase {
    private CANSparkMax m_wristMotor;
    private double lastSpeed = 0;

public Wrist() {
    m_wristMotor = new CANSparkMax(Constants.CanBus.wristMotor, MotorType.kBrushless);
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
    m_wristMotor.set(0);
  }

  public void setWristSpeed(double speed) {
    lastSpeed = speed;
    m_wristMotor.set(speed);  
  }

public void periodic() {
    Logger.recordOutput("Wrist Temp", m_wristMotor.getMotorTemperature());
    Logger.recordOutput("Wrist RPM", m_wristMotor.getVoltageCompensationNominalVoltage());
  }
}