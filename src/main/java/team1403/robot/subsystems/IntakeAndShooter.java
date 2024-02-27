package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.AdvancedMotorController.CougarIdleMode;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.CougarTalonFx;
import team1403.robot.Constants;

/**
 * creating the intake and shooter class.
 */
public class IntakeAndShooter extends SubsystemBase {

  // constant that should be in constants >:(
  private double lastSpeed = 0;
  
  // Intake motor
  private static CougarSparkMax m_intakeMotor;
  
  // shooter motors
  private CougarTalonFx m_shooterMotorTop;
  private CougarTalonFx m_shooterMotorBottom;

  // photogates
  private DigitalInput m_intakePhotogate;
  private DigitalInput m_shooterPhotogate;
  private LinearFilter m_shooterFilter = LinearFilter.movingAverage(2);
  private LinearFilter m_intakeFilter = LinearFilter.movingAverage(2);
  private PIDController m_bottomShooter;
  private PIDController m_topShooter;


  /**
   * creating shooter devices.
   *
   * @param injectedParameters
   */
  /**
   * creating motor and sensors for the intake.
   *
   * @param injectedParameters injected parameters.
   */
  public IntakeAndShooter() {
    // intake motors and sensors
    m_intakeMotor = CougarSparkMax.makeBrushless("Top Intake Motor", Constants.CanBus.intakeMotorID,
        SparkRelativeEncoder.Type.kHallSensor);
    m_intakePhotogate = new DigitalInput(Constants.RioPorts.intakePhotogate1);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    // shooter motors and sensors
    m_shooterMotorTop = new CougarTalonFx("Top Shooter Motor", Constants.CanBus.shooterMotorTopID);
    m_shooterMotorBottom = new CougarTalonFx("Bottom Shooter Motor", Constants.CanBus.shooterMotorBottomID);
    m_shooterPhotogate = new DigitalInput(Constants.RioPorts.shooterPhotogate);
    m_shooterMotorTop.setIdleMode(CougarIdleMode.BRAKE);
    m_shooterMotorBottom.setIdleMode(CougarIdleMode.BRAKE);
    m_bottomShooter = new PIDController(0.00001, 0.0 , 0.0);
    m_topShooter = new PIDController(0.00001, 0.0, 0.0);

    m_shooterMotorTop.getEmbeddedEncoder().setVelocityConversionFactor(60.);
    m_shooterMotorBottom.getEmbeddedEncoder().setVelocityConversionFactor(60.);
  }

  /**
   * state of the intake.
   *
   * @return true or false depending on if it is trigered.
   */
  public boolean isIntakePhotogateTriggered() {
    return !m_intakePhotogate.get();
  }

  /**
   * if the intake is ready.
   *
   * @return true or false
   */
  public boolean intakeReady() {
    if (lastSpeed == Math.abs(m_intakeMotor.getEncoder().getVelocity())
        && lastSpeed == Math.abs(m_intakeMotor.getEncoder().getVelocity())) {
      return true;
    } else {
      return false;
    }
  }
  
  /**
   * stopping intake.
   */
  public void intakeStop() {
    m_intakeMotor.setSpeed(0);
  }

  /**
   * setting intake speed
   *
   * @param speed
   */
  public void setIntakeSpeed(double speed) {
    // if (m_intakeLimitSwitch.get()) {
    // stop();
    // return;
    // }
    lastSpeed = speed;
    m_intakeMotor.set(speed);
    // if there is an error when testing (note doesn't get taken in) try changing
    // the direction of the motor

  }
  public boolean shooterPhotogateCheckTrigger() {
    int value = isShooterPhotogateTriggered() ? 1 : 0;
    return m_shooterFilter.calculate(value) > .5;
  }
    public boolean intakePhotogateCheckTrigger() {
    int value = isIntakePhotogateTriggered() ? 1 : 0;
    return m_intakeFilter.calculate(value) > .5;
  }
  /**
   * is shooter photogate triggered.
   *
   * @return state of the shooter photogate.
   */
  public boolean isShooterPhotogateTriggered() {
    return !m_shooterPhotogate.get();
  }

  /**
   * if the shooter is ready.
   */
  public boolean shooterReady() {
    if (lastSpeed == Math.abs(m_shooterMotorTop.getEmbeddedEncoder().getVelocityValue())
        && lastSpeed == Math.abs(m_shooterMotorBottom.getEmbeddedEncoder().getVelocityValue())) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Checking if the speeds of the top ond bottom shooter motor are the same.
   *
   * @return true or false.
   */
  public boolean shooterSpeedIsEqual() {
    if (Math.abs(m_shooterMotorTop.getEmbeddedEncoder().getVelocityValue()) == Math
        .abs(m_shooterMotorBottom.getEmbeddedEncoder().getVelocityValue())) {
      return true;
    } else {
      return false;
    }
  }

  public void setShooterRPM(double rpm) {
    m_bottomShooter.setSetpoint(rpm);
    m_topShooter.setSetpoint(rpm);
  }
  /**
   * Stopping the shooter motors.
   */
  public void shooterStop() {
    m_bottomShooter.setSetpoint(0);
        m_topShooter.setSetpoint(0);
    m_shooterMotorTop.setSpeed(0);
    m_shooterMotorBottom.setSpeed(0);
  }

  /**
   * setting the shooter speed.
   * 
   * @param speed
   */
  public void setShooterSpeed(double speed) {
    lastSpeed = speed;
    m_shooterMotorTop.setSpeed(-speed);
    m_shooterMotorBottom.setSpeed(-speed);
    // if there is an error when testing (note doesn't get shot out) try changing
    // the direction of the motor

  }

  public void periodic() {
    Logger.recordOutput("Intake Top Motor Temp", m_intakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Intake Top Motor RPM", -m_shooterMotorTop.getEmbeddedEncoder().getVelocityValue());
    SmartDashboard.putNumber("RPM setpoint",  m_topShooter.getSetpoint());
    SmartDashboard.putBoolean("Shooter Ready", Math.abs(m_bottomShooter.getSetpoint() + m_shooterMotorBottom.getEmbeddedEncoder().getVelocityValue()) < m_bottomShooter.getSetpoint() / 0.05);
    m_shooterMotorBottom.setSpeed(m_shooterMotorBottom.get() - m_bottomShooter.calculate(-m_shooterMotorBottom.getEmbeddedEncoder().getVelocityValue()));
    m_shooterMotorTop.setSpeed(m_shooterMotorTop.get() - m_topShooter.calculate(-m_shooterMotorTop.getEmbeddedEncoder().getVelocityValue()));
  }
}
