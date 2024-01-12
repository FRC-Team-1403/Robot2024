package team1403.robot.subsystems;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;

public class Shooter extends SubsystemBase {

  private CougarSparkMax m_shooterMotor;
  private WpiLimitSwitch m_shooterLimitSwitch;


  public Shooter(CougarLibInjectedParameters injectedParameters) {
    m_shooterMotor = CougarSparkMax.makeBrushless(
      "Shooter Motor", Constants.CanBus.hangerMotor, SparkRelativeEncoder.Type.kHallSensor);

    m_shooterLimitSwitch = new WpiLimitSwitch(
      "Shooter Limit Switch", Constants.RioPorts.shooterLimitSwitchPort);
  }

  public void setShooterSpeed(double speed) {
    m_shooterMotor.set(speed);
  }
}
