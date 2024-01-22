package team1403.robot.subsystems;

import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;

public class Hanger extends SubsystemBase{
  private WpiLimitSwitch m_hangerLimitSwitchTop;
    private WpiLimitSwitch m_hangerLimitSwitchBottom;
  private CougarSparkMax m_hangerMotor;

  public Hanger(CougarLibInjectedParameters injectedParameters) {
    m_hangerMotor = CougarSparkMax.makeBrushless(
      "Hanger Motor", Constants.CanBus.hangerMotor, SparkRelativeEncoder.Type.kHallSensor);
  m_hangerLimitSwitchTop = new WpiLimitSwitch("limit switch Top", Constants.Hanger.channel);
  m_hangerLimitSwitchBottom = new WpiLimitSwitch("limit switch Bottom", Constants.Hanger.channel);
  }

  public void setHangerSpeed(double speed) {
    m_hangerMotor.set(speed);
  }

public void ifLimitHit() {
  
  if (isAtTop() == true || isAtBottom() == true) {

    m_hangerMotor.set(0);

  } else {

    m_hangerMotor.set(3);

  }

}

    public boolean isAtTop() {

    return m_hangerLimitSwitchTop.get();

  }

    public boolean isAtBottom() {

    return m_hangerLimitSwitchBottom.get();
    }

}
//top limit switch: go up until hits the top; bottom limit switch: down until hits the bottom (at - speed)