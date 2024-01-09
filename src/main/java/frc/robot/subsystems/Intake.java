package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.CougarLibInjectedParameters;
import frc.lib.device.LimitSwitch;
import frc.lib.device.wpi.CougarSparkMax;
import frc.lib.device.wpi.WpiLimitSwitch;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

  private CougarSparkMax m_intakeMotor;
  private frc.lib.device.wpi.WpiLimitSwitch m_intakeLimitSwitch;

  public Intake(CougarLibInjectedParameters injectedParameters) {
    m_intakeMotor = CougarSparkMax.makeBrushless(
      "Intake Motor", Constants.CanBus.intakeMotor, Type.kHallSensor, null);

    m_intakeLimitSwitch = new WpiLimitSwitch(
      "Intake Limit Switch", Constants.RioPorts.intakeLimitSwitchPort);
  }

  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(speed);
  }
}
