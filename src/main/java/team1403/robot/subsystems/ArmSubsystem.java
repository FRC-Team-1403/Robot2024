package team1403.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
//  private final DutyCycleEncoder m_armAbsoluteEncoderTwo;
  private final PIDController m_pivotPid;
  private double m_pivotAngleSetpoint;

  /**
   * Initializing the arn subsystem.
   *
   * @param injectedParameters Cougar injected parameters.
   */
  public ArmSubsystem() {

    m_pivotMotor = new CANSparkMax(Constants.CanBus.m_pivotMotor, MotorType.kBrushless);
    m_armAbsoluteEncoder = new DutyCycleEncoder(Constants.RioPorts.kArmAbsoluteEncoder);
//    m_armAbsoluteEncoderTwo = new DutyCycleEncoder(Constants.RioPorts.kArmAbsoluteEncoder);

    new WpiLimitSwitch("maxArmLimitSwitch",
        Constants.RioPorts.kArmLimitSwitch);

    m_pivotPid = new PIDController(Constants.Arm.kPArmPivot, Constants.Arm.kPArmPivot, Constants.Arm.kPArmPivot);
    
  }

  // --------------------------- General methods ---------------------------

  /**
   * Sets the setpoints for the pivot, extension, wrist, and intake to go to.
   *
   * @param pivotAngle      the pivot angle.
   */
  public void moveArm(double pivotAngle) {
    this.m_pivotAngleSetpoint = pivotAngle;
  }

  @Override
  public void periodic() {
    m_pivotMotor.set(
        MathUtil.clamp(
            m_pivotPid.calculate(
                m_armAbsoluteEncoder.getAbsolutePosition(), m_pivotAngleSetpoint), 
                -1,
                 1));
  }

  /**
   * Returns the object for the pivot motor.
   * 
   * @return the pivot motor.
   */
  public CANSparkMax getPivotMotor() {
    return m_pivotMotor;
  }
}