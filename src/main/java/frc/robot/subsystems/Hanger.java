package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.CougarLibInjectedParameters;
import frc.lib.device.LimitSwitch;
import frc.lib.device.wpi.CougarSparkMax;
import frc.lib.device.wpi.WpiLimitSwitch;
import frc.robot.Constants;

public class Hanger extends SubsystemBase{

  private CougarSparkMax m_hangerMotor;

  public Hanger(CougarLibInjectedParameters injectedParameters) {
    m_hangerMotor = CougarSparkMax.makeBrushless(
      "Hanger Motor", Constants.CanBus.hangerMotor, Type.kHallSensor, null);
  }

  public void setHangerSpeed(double speed) {
    m_hangerMotor.set(speed);
  }
}
