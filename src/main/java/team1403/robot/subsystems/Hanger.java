package team1403.robot.subsystems;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;

public class Hanger extends SubsystemBase{
  private WpiLimitSwitch m_hangerLimitSwitchTop;
  private WpiLimitSwitch m_hangerLimitSwitchBottom;
  private CougarSparkMax m_rightHangerMotor;
  private CougarSparkMax m_leftHangerMotor;


  public Hanger(CougarLibInjectedParameters injectedParameters) {
    m_rightHangerMotor = CougarSparkMax.makeBrushless("Right Hanger Motor", Constants.CanBus.rightHangerMotor, SparkRelativeEncoder.Type.kHallSensor);
    m_leftHangerMotor = CougarSparkMax.makeBrushless("Left Hanger Motor", Constants.CanBus.leftHangerMotor, SparkRelativeEncoder.Type.kHallSensor);
    m_hangerLimitSwitchTop = new WpiLimitSwitch("limit switch Top", Constants.Hanger.channel);
    m_hangerLimitSwitchBottom = new WpiLimitSwitch("limit switch Bottom", Constants.Hanger.channel);
  }

  public void setHangerSpeed(double speed) {
    m_rightHangerMotor.set(speed);
    m_leftHangerMotor.set(speed);
  }

public void ifLimitHit() {
  if (isAtTop() == true || isAtBottom() == true) {
    m_rightHangerMotor.set(0);
    m_leftHangerMotor.set(0);
  } else {
    m_rightHangerMotor.set(3);
    m_leftHangerMotor.set(3);
  }
}

    public boolean isAtTop() {
      return m_hangerLimitSwitchTop.get();
    }

    public boolean isAtBottom() {
      return m_hangerLimitSwitchBottom.get();
    }

    public void periodic() {
      SmartDashboard.putNumber("Hanger Temp", m_rightHangerMotor.getMotorTemperature());
      SmartDashboard.putNumber("Hanger RPM", m_rightHangerMotor.getVoltageCompensationNominalVoltage());
      SmartDashboard.putNumber("Hanger Temp", m_leftHangerMotor.getMotorTemperature());
      SmartDashboard.putNumber("Hanger RPM", m_leftHangerMotor.getVoltageCompensationNominalVoltage());
    }
}
//top limit switch: go up until hits the top; bottom limit switch: down until hits the bottom (at - speed)