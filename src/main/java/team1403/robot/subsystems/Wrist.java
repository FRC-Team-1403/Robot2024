package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants; 

public class Wrist extends SubsystemBase {
    
    private CANSparkMax m_wristMotor;

    public Wrist() {
        m_wristMotor = new CANSparkMax(Constants.CanBus.wristMotor, MotorType.kBrushless);
    }

    public void stop() {
        m_wristMotor.set(0);
    }

    public void periodic() {
        Logger.recordOutput("Wrist Temp", m_wristMotor.getMotorTemperature());
  }
}