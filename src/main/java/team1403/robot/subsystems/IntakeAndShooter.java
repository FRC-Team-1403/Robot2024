package team1403.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import team1403.lib.util.CougarLogged;
import team1403.robot.Constants;

/**
 * creating the intake and shooter class.
 */
public class IntakeAndShooter extends SubsystemBase implements CougarLogged {  
  // Intake motor
  private static CANSparkMax m_intakeMotor;
  
  // shooter motors
  private TalonFX m_shooterMotorTop;
  private TalonFX m_shooterMotorBottom;
  private StatusSignal<Double> m_topVel;
  private StatusSignal<Double> m_bottomVel;

  // photogates
  private DigitalInput m_intakePhotogate;
  private DigitalInput m_shooterPhotogate;

  public boolean isLoaded;

  private final MotionMagicVelocityDutyCycle m_request = new MotionMagicVelocityDutyCycle(0);


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
    m_intakeMotor = new CANSparkMax(Constants.CanBus.intakeMotorID, MotorType.kBrushless);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_intakePhotogate = new DigitalInput(Constants.RioPorts.intakePhotogate1);
    // shooter motors and sensors
    m_shooterMotorTop = new TalonFX(Constants.CanBus.shooterMotorTopID);
    m_shooterMotorBottom = new TalonFX(Constants.CanBus.shooterMotorBottomID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotionMagic.MotionMagicAcceleration = 2000; // RPM/s -> ~0.5 s to max

    // FIXME: Tune these values!
    var slot0Configs = config.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0.0092;
    slot0Configs.kA = 0;
    slot0Configs.kP = 0.003;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    m_shooterMotorTop.getConfigurator().apply(config);
    m_shooterMotorBottom.getConfigurator().apply(config);

    m_shooterPhotogate = new DigitalInput(Constants.RioPorts.shooterPhotogate);

    m_topVel = m_shooterMotorTop.getVelocity();
    m_bottomVel = m_shooterMotorBottom.getVelocity();

    if(Constants.DEBUG_MODE) {
      Constants.kDebugTab.addBoolean("Intake Sensor", () -> isIntakePhotogateTriggered());
      Constants.kDebugTab.addBoolean("Shooter Sensor", () -> isShooterPhotogateTriggered());
      Constants.kDebugTab.addBoolean("Shooter (teleop) Ready", () -> teleopIsReady());
    }
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
   * stopping intake.
   */
  public void intakeStop() {
    m_intakeMotor.set(0);
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
    return !m_shooterPhotogate.get();
  }

  public void applySetpoint(SonicBlasterSetpoint setpoint) {
    setIntakeSpeed(setpoint.getIntakeSpeed());
    setShooterRPM(setpoint.getShooterRPM());
  }

  public void setShooterRPM(double rpm) {
    m_request.Velocity = rpm / 60.0;
  }

  /**
   * Stopping the shooter motors.
   */
  public void shooterStop() {
    m_request.Velocity = 0;
  }

  public boolean isReady(){
    return Math.abs(m_request.Velocity * 60 - m_bottomVel.getValue() * 60) < 300 && 
           Math.abs(m_request.Velocity * 60 - m_topVel.getValue() * 60) < 300;
  }

  public boolean teleopIsReady() {
    return Math.abs(m_request.Velocity * 60 - m_bottomVel.getValue() * 60) < 1000;
  }

  public void periodic() {
    m_topVel.refresh();
    m_bottomVel.refresh();
    m_shooterMotorTop.setControl(m_request);
    m_shooterMotorBottom.setControl(m_request);

    Constants.IntakeAndShooter.isLoaded = (isIntakePhotogateTriggered() && isShooterPhotogateTriggered());
    Blackbox.setLoaded(isIntakePhotogateTriggered() && !isShooterPhotogateTriggered());

    // set to intake
    if (!isLoaded) {
      setIntakeSpeed(0.3);
      shooterStop();
    }
    if (isIntakePhotogateTriggered() && isShooterPhotogateTriggered() 
        && (m_topVel.getValue() > 10) && (m_bottomVel.getValue() > 10)) {
      setIntakeSpeed(-0.1);
    }
    if (isIntakePhotogateTriggered() && !isShooterPhotogateTriggered() 
        && (m_topVel.getValue() < 10) && (m_bottomVel.getValue() < 10)) {
      intakeStop();
    }
    if ((m_topVel.getValue() > 900) && (m_bottomVel.getValue() > 900)) {
      setIntakeSpeed(0.5);
    }

    log("Intake/Motor Temp", m_intakeMotor.getMotorTemperature());
    log("Shooter/Speed", m_shooterMotorTop.get());
    log("Shooter/Voltage", m_shooterMotorTop.getMotorVoltage().getValueAsDouble());
    log("Shooter/gate", isShooterPhotogateTriggered());
    log("Intake/gate", isIntakePhotogateTriggered());
    log("Shooter/top Motor RPM", m_topVel.getValue());
    log("Shooter/bottom Motor RPM", m_bottomVel.getValue());
    // log("Intake/RPM", m_intakeMotor.get());
    log("Intake/Speed Setpoint", m_intakeMotor.get());
    log("Shooter/RPM setpoint",  m_request.Velocity);
    log("Intake/Current", m_intakeMotor.getOutputCurrent());
    log("Shooter/top current", m_shooterMotorTop.getStatorCurrent().getValueAsDouble());
    log("Shooter/bottom current", m_shooterMotorBottom.getStatorCurrent().getValueAsDouble());
  }
}
