package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;

public class Hanger extends SubsystemBase{
  private WpiLimitSwitch m_hangerLimitSwitchTop;
    private WpiLimitSwitch m_hangerLimitSwitchBottom;
  private CougarSparkMax m_definiteHangerMotor;
  private CougarSparkMax m_possibleHangerMotor;


  public Hanger(CougarLibInjectedParameters injectedParameters) {
    m_definiteHangerMotor = CougarSparkMax.makeBrushless("Definite Hanger Motor", Constants.CanBus.definiteHangerMotor, SparkRelativeEncoder.Type.kHallSensor);
    m_possibleHangerMotor = CougarSparkMax.makeBrushless("Possible Hanger Motor", Constants.CanBus.possibleHangerMotor, null);


  m_hangerLimitSwitchTop = new WpiLimitSwitch("limit switch Top", Constants.Hanger.channel);
  m_hangerLimitSwitchBottom = new WpiLimitSwitch("limit switch Bottom", Constants.Hanger.channel);
  }

  public void setHangerSpeed(double speed) {
    m_definiteHangerMotor.set(speed);
    m_possibleHangerMotor.set(speed);
  }

public void ifLimitHit() {
  
  if (isAtTop() == true || isAtBottom() == true) {

    m_definiteHangerMotor.set(0);
    m_possibleHangerMotor.set(0);

  } else {

    m_definiteHangerMotor.set(3);
    m_possibleHangerMotor.set(3);

  }

}

    public boolean isAtTop() {

    return m_hangerLimitSwitchTop.get();

  }

    public boolean isAtBottom() {

    return m_hangerLimitSwitchBottom.get();
    }

    public void periodic() {
      Logger.recordOutput("Hanger Temp", m_definiteHangerMotor.getMotorTemperature());
      Logger.recordOutput("Hanger RPM", m_definiteHangerMotor.getVoltageCompensationNominalVoltage());
      Logger.recordOutput("Hanger Temp", m_possibleHangerMotor.getMotorTemperature());
      Logger.recordOutput("Hanger RPM", m_possibleHangerMotor.getVoltageCompensationNominalVoltage());
      
    }
}
//top limit switch: go up until hits the top; bottom limit switch: down until hits the bottom (at - speed)