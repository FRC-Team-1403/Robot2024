package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.deser.DeserializationProblemHandler;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
  // Intake motor
  private static CougarSparkMax m_intakeMotor;
  
  // shooter motors
  private CougarTalonFx m_shooterMotorTop;
  private CougarTalonFx m_shooterMotorBottom;

  // photogates
  private DigitalInput m_intakePhotogate;
  private DigitalInput m_shooterPhotogate;
  private PIDController m_bottomShooter;
  private PIDController m_topShooter;

  private Debouncer m_shooterDebouncer;
  private Debouncer m_intakeDebouncer;


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
    m_bottomShooter = new PIDController(0.000011, 0.0 , 0.0);
    m_topShooter = new PIDController(0.000011, 0.0, 0.0);
    m_shooterDebouncer = new Debouncer(0.03, DebounceType.kBoth);
    m_intakeDebouncer = new Debouncer(0.04, DebounceType.kBoth);

    m_shooterMotorTop.getEmbeddedEncoder().setVelocityConversionFactor(60.);
    m_shooterMotorBottom.getEmbeddedEncoder().setVelocityConversionFactor(60.);
  }

  /**
   * state of the intake.
   *
   * @return true or false depending on if it is trigered.
   */
  public boolean isIntakePhotogateTriggered() {
    return m_intakeDebouncer.calculate(!m_intakePhotogate.get());
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
    m_intakeMotor.set(speed);
    // if there is an error when testing (note doesn't get taken in) try changing
    // the direction of the motor

  }

  /**
   * is shooter photogate triggered.
   *
   * @return state of the shooter photogate.
   */
  public boolean isShooterPhotogateTriggered() {
    return m_shooterDebouncer.calculate(!m_shooterPhotogate.get());
  }

  public void setShooterRPM(double rpm) {
    m_bottomShooter.setSetpoint(rpm);
    m_topShooter.setSetpoint(rpm);
  }

  public double getShooterRPM(){
    return m_bottomShooter.getSetpoint();
  }

  /**
   * Stopping the shooter motors.
   */
  public void shooterStop() {
    m_bottomShooter.setSetpoint(0);
    m_topShooter.setSetpoint(0);
  }

  /**
   * setting the shooter speed.
   * 
   * @param speed
   */
  public void setShooterSpeed(double speed) {
    m_shooterMotorTop.setSpeed(-speed);
    m_shooterMotorBottom.setSpeed(-speed);
    // if there is an error when testing (note doesn't get shot out) try changing
    // the direction of the motor

  }

  public boolean isReady(){
    return Math.abs(m_bottomShooter.getSetpoint() + m_shooterMotorBottom.getEmbeddedEncoder().getVelocityValue()) < 300;
  }

  public boolean teleopIsReady() {
    return Math.abs(m_bottomShooter.getSetpoint() + m_shooterMotorBottom.getEmbeddedEncoder().getVelocityValue()) < 1000;
  }
  
  public void periodic() {
    Logger.recordOutput("Intake Motor Temp", m_intakeMotor.getMotorTemperature());
    SmartDashboard.putNumber("Intake Motor RPM", m_intakeMotor.getEmbeddedEncoder().getVelocityValue());
    SmartDashboard.putNumber("Shooter Top Motor RPM", -m_shooterMotorTop.getEmbeddedEncoder().getVelocityValue());
    SmartDashboard.putNumber("RPM setpoint",  m_topShooter.getSetpoint());
    SmartDashboard.putBoolean("Intake Sensor", isIntakePhotogateTriggered());
    SmartDashboard.putBoolean("Shooter Sensor", isShooterPhotogateTriggered());
    SmartDashboard.putBoolean("Shooter Ready", isReady());
    Logger.recordOutput("Shooter Speed", m_shooterMotorTop.get());
    Logger.recordOutput("Shooter Voltage", m_shooterMotorTop.geTalonFxApi().getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Shooter gate", !isShooterPhotogateTriggered());
    Logger.recordOutput("Intake gate", isIntakePhotogateTriggered());
    Logger.recordOutput("Shooter top Motor RPM", -m_shooterMotorTop.getEmbeddedEncoder().getVelocityValue());
    Logger.recordOutput("Intake RPM", m_intakeMotor.getEmbeddedEncoder().getVelocityValue());
    Logger.recordOutput("Intake Speed Setpoint", m_intakeMotor.get());
    Logger.recordOutput("Shooter RPM setpoint",  m_topShooter.getSetpoint());
    Logger.recordOutput("Shooter Bottom Motor RPM", -m_shooterMotorBottom.getEmbeddedEncoder().getVelocityValue());

    m_shooterMotorBottom.setSpeed(m_shooterMotorBottom.get() - m_bottomShooter.calculate(-m_shooterMotorBottom.getEmbeddedEncoder().getVelocityValue()));
    m_shooterMotorTop.setSpeed(m_shooterMotorTop.get() - m_topShooter.calculate(-m_shooterMotorTop.getEmbeddedEncoder().getVelocityValue()));
  }
}
