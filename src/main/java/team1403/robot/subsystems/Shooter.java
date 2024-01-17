package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;

public class Shooter extends SubsystemBase {

  private CougarSparkMax m_shooterMotorTop;
  private CougarSparkMax m_shooterMotorBottom;
  private double lastSpeed = 0;

  public Shooter(CougarLibInjectedParameters injectedParameters) {
    m_shooterMotorTop = CougarSparkMax.makeBrushless(
      "Top Shooter Motor", Constants.CanBus.shooterMotorTop, SparkRelativeEncoder.Type.kHallSensor);
    m_shooterMotorBottom = CougarSparkMax.makeBrushless(
      "Bottom Shooter Motor", Constants.CanBus.shooterMotorBottom, SparkRelativeEncoder.Type.kHallSensor);

  }
  public boolean ready() {
    if (lastSpeed == m_shooterMotorTop.getEncoder().getVelocity() && lastSpeed == m_shooterMotorBottom.getEncoder().getVelocity()) {
      return true;
    }
    else {
      return false;
    }
  }

  public boolean speedIsEqual() {

    if (m_shooterMotorTop.getEncoder().getVelocity() == m_shooterMotorBottom.getEncoder().getVelocity()) {

        return true;

    } else {

        return false;

    }

  }

public void stop(){
  m_shooterMotorTop.setSpeed(0);
  m_shooterMotorBottom.setSpeed(0);
}

  public void setShooterSpeed(double speed) {
    lastSpeed = speed;
    m_shooterMotorTop.set(speed);
    m_shooterMotorBottom.set(speed);
  }
  public void periodic() {
    Logger.recordOutput("Shooter Temp", m_shooterMotorTop.getMotorTemperature());
    Logger.recordOutput("Shooter Temp", m_shooterMotorBottom.getMotorTemperature());

  }
}
