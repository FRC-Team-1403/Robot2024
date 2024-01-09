package team1403.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.LimitSwitch;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;

public class Shooter extends SubsystemBase{

  private CougarSparkMax m_shooterMotor;
  private team1403.lib.device.wpi.WpiLimitSwitch m_shooterLimitSwitch;


  public Shooter(CougarLibInjectedParameters injectedParameters) {
    m_shooterMotor = CougarSparkMax.makeBrushless(
      "Shooter Motor", Constants.CanBus.hangerMotor, Type.kHallSensor, null);

    m_shooterLimitSwitch = new WpiLimitSwitch(
      "Shooter Limit Switch", Constants.RioPorts.shooterLimitSwitchPort);
  }

  public void setShooterSpeed(double speed)
  {
    m_shooterMotor.set(speed);
  }
}
