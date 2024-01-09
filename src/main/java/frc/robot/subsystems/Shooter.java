package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.CougarLibInjectedParameters;
import frc.lib.device.LimitSwitch;
import frc.lib.device.wpi.CougarSparkMax;
import frc.lib.device.wpi.WpiLimitSwitch;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{

  private CougarSparkMax m_shooterMotor;
  private frc.lib.device.wpi.WpiLimitSwitch m_shooterLimitSwitch;


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
