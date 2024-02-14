package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.AdvancedMotorController;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;

public class Hanger extends SubsystemBase {
  private WpiLimitSwitch m_hangerLimitSwitchTop;
  private WpiLimitSwitch m_hangerLimitSwitchBottom;
  private CougarSparkMax m_definiteHangerMotor;
  private CougarSparkMax m_possibleHangerMotor;


  public Hanger(CougarLibInjectedParameters injectedParameters) {
    m_definiteHangerMotor = CougarSparkMax.makeBrushless("Left Hanger Motor", Constants.CanBus.leftHangerMotorID, SparkRelativeEncoder.Type.kHallSensor);
    m_possibleHangerMotor = CougarSparkMax.makeBrushless("Right Hanger Motor", Constants.CanBus.rightHangerMotorID, SparkRelativeEncoder.Type.kHallSensor);

    m_possibleHangerMotor.follow((AdvancedMotorController)m_definiteHangerMotor);

    m_hangerLimitSwitchTop = new WpiLimitSwitch("limit switch Top", Constants.Hanger.channel);
    m_hangerLimitSwitchBottom = new WpiLimitSwitch("limit switch Bottom", Constants.Hanger.channel);
  }

  public void setHangerSpeed(double speed) {
    m_definiteHangerMotor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void runHanger() {
    if (isAtTop() || isAtBottom()) {
      setHangerSpeed(0);
    } else {
      setHangerSpeed(1);
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