package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;

/**
 * creating the intake and shooter class.
 */
public class IntakeAndShooter extends SubsystemBase {  
  // Intake motor
  private static CougarSparkMax m_intakeMotor;
  
  // shooter motors
  private TalonFX m_shooterMotorTop;
  private TalonFX m_shooterMotorBottom;
  private StatusSignal<Double> m_topVel;
  private StatusSignal<Double> m_bottomVel;

  // photogates
  private DigitalInput m_intakePhotogate;
  private DigitalInput m_shooterPhotogate;

  private Debouncer m_shooterDebouncer;
  private Debouncer m_intakeDebouncer;

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
    m_intakeMotor = CougarSparkMax.makeBrushless("Top Intake Motor", Constants.CanBus.intakeMotorID,
        SparkRelativeEncoder.Type.kHallSensor);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_intakePhotogate = new DigitalInput(Constants.RioPorts.intakePhotogate1);
    // shooter motors and sensors
    m_shooterMotorTop = new TalonFX(Constants.CanBus.shooterMotorTopID);
    m_shooterMotorBottom = new TalonFX(Constants.CanBus.shooterMotorBottomID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Rotations per second to Rotations per minute
    config.Feedback.SensorToMechanismRatio = 60.;
    config.MotionMagic.MotionMagicAcceleration = 12000; // RPM/s -> ~0.5 s to max

    // FIXME: Tune these values!
    var slot0Configs = config.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 1.0 / 6300;
    slot0Configs.kA = 0;
    slot0Configs.kP = 0.00005;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    m_shooterMotorTop.getConfigurator().apply(config);
    m_shooterMotorBottom.getConfigurator().apply(config);

    m_shooterPhotogate = new DigitalInput(Constants.RioPorts.shooterPhotogate);
    m_shooterDebouncer = new Debouncer(0.02, DebounceType.kBoth);
    m_intakeDebouncer = new Debouncer(0.02, DebounceType.kBoth);

    m_intakeMotor.setSmartCurrentLimit(Constants.IntakeAndShooter.kIntakeCurrentLimit);

    m_topVel = m_shooterMotorTop.getVelocity();
    m_bottomVel = m_shooterMotorBottom.getVelocity();

    Constants.kDebugTab.addBoolean("Intake Sensor", () -> isIntakePhotogateTriggered());
    Constants.kDebugTab.addBoolean("Shooter Sensor", () -> isShooterPhotogateTriggered());
    Constants.kDebugTab.addBoolean("Shooter Ready", () -> isReady());
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
    m_request.Velocity = rpm;
  }

  /**
   * Stopping the shooter motors.
   */
  public void shooterStop() {
    m_request.Velocity = 0;
  }

  public boolean isReady(){
    return Math.abs(m_request.Velocity - m_bottomVel.getValue()) < 300 && 
           Math.abs(m_request.Velocity - m_topVel.getValue()) < 300;
  }

  public boolean teleopIsReady() {
    return Math.abs(m_request.Velocity - m_bottomVel.getValue()) < 1000;
  }

  public void periodic() {
    m_topVel.refresh();
    m_bottomVel.refresh();
    m_shooterMotorTop.setControl(m_request);
    m_shooterMotorBottom.setControl(m_request);

    Logger.recordOutput("Intake Motor Temp", m_intakeMotor.getMotorTemperature());
    Logger.recordOutput("Shooter Speed", m_shooterMotorTop.get());
    Logger.recordOutput("Shooter Voltage", m_shooterMotorTop.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Shooter gate", isShooterPhotogateTriggered());
    Logger.recordOutput("Intake gate", isIntakePhotogateTriggered());
    Logger.recordOutput("Shooter top Motor RPM", m_topVel.getValue());
    Logger.recordOutput("Intake RPM", m_intakeMotor.getEmbeddedEncoder().getVelocityValue());
    Logger.recordOutput("Intake Speed Setpoint", m_intakeMotor.get());
    Logger.recordOutput("Shooter RPM setpoint",  m_request.Velocity);
    Logger.recordOutput("Intake current", m_intakeMotor.getOutputCurrent());
    Logger.recordOutput("Shooter top current", m_shooterMotorTop.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter bottom current", m_shooterMotorBottom.getStatorCurrent().getValueAsDouble());
  }
}
