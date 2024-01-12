package team1403.robot.subsystems;

import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;

public class Hanger extends SubsystemBase{

  private CougarSparkMax m_hangerMotor;

  public Hanger(CougarLibInjectedParameters injectedParameters) {
    m_hangerMotor = CougarSparkMax.makeBrushless(
      "Hanger Motor", Constants.CanBus.hangerMotor, SparkRelativeEncoder.Type.kHallSensor);
  }

  public void setHangerSpeed(double speed) {
    m_hangerMotor.set(speed);
  }
}
