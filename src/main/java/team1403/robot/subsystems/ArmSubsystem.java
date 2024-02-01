package team1403.robot.subsystems;

import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.WpiLimitSwitch;
import team1403.robot.Constants;

/**
 * Class creating the arm subsystem. The subsystem takes care of moving the arm
 * to a specified position.
 * The specified position can be changed via the moveArm() method.
 * 
 */
public class ArmSubsystem extends SubsystemBase {
  // Arm
  private final CANSparkMax m_pivotMotor;
  private final DutyCycleEncoder m_armAbsoluteEncoder;
  private final PIDController m_pivotPid;
  private final DigitalInput m_ArmLimitSwitch;
  private double m_pivotAngleSetpoint;
  private double m_tolerance;

  /**
   * Initializing the arm subsystem.
   *
   * @param injectedParameters Cougar injected parameters.
   */
  private WpiLimitSwitch m_limitSwitch;
  public ArmSubsystem() {
    m_ArmLimitSwitch = new DigitalInput(Constants.RioPorts.shooterPhotogate);
    m_pivotMotor = new CANSparkMax(Constants.CanBus.m_pivotMotor, MotorType.kBrushless);
    m_armAbsoluteEncoder = new DutyCycleEncoder(Constants.RioPorts.kArmAbsoluteEncoder);

    m_limitSwitch = new WpiLimitSwitch("maxArmLimitSwitch",
        Constants.RioPorts.kArmLimitSwitch);
    m_pivotPid = new PIDController(Constants.Arm.kPArmPivot, Constants.Arm.kPArmPivot, Constants.Arm.kPArmPivot);
    
  }

  // --------------------------- General methods ---------------------------

  /**
   * Sets the setpoints for the pivot, extension, wrist, and intake to go to.
   *
   * @param pivotAngle      the pivot angle.
   */
  
  public void setArmSetpoint(double pivotAngle) {
    m_pivotAngleSetpoint = pivotAngle;
    if (m_ArmLimitSwitch.get()) {
      stop();
    }
  }

  public boolean getArmLimitSwitch() {
    return m_ArmLimitSwitch.get();
  }
  @Override
  public void periodic() {
      m_pivotMotor.set(MathUtil.clamp(m_pivotPid.calculate(m_armAbsoluteEncoder.get(), m_pivotAngleSetpoint), -1, 1));
    Logger.recordOutput("Arm Angle Setpoint", m_pivotAngleSetpoint);
    Logger.recordOutput("Pivot Motor RPM", getPivotMotorSpeed());
    Logger.recordOutput("Arm Limitswitch", getArmLimitSwitch());
    // run the arm if limit switch not pressed 
      if (!m_limitSwitch.get()) 
      m_pivotMotor.set(
        MathUtil.clamp(
            m_pivotPid.calculate(
                m_armAbsoluteEncoder.getAbsolutePosition(), m_pivotAngleSetpoint), 
                -1,
                 1));
  }

  public double getPivotMotorSpeed() {
    return m_pivotMotor.get();
  }

  public void stop() {
    m_pivotMotor.set(0);
  }
}