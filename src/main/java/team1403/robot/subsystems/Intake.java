package team1403.robot.subsystems;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;

public class Intake extends SubsystemBase{

  private CougarSparkMax m_intakeMotor;
  private WpiLimitSwitch m_intakeLimitSwitch;

  public Intake(CougarLibInjectedParameters injectedParameters) {
    m_intakeMotor = CougarSparkMax.makeBrushless(
      "Intake Motor", Constants.CanBus.intakeMotor, SparkRelativeEncoder.Type.kHallSensor);

    m_intakeLimitSwitch = new WpiLimitSwitch(
      "Intake Limit Switch", Constants.RioPorts.intakeLimitSwitchPort);
  }

  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(speed);
  }
}
