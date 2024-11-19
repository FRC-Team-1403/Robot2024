package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.robot.Constants;

/**
 * Class creating the arm subsystem. The subsystem takes care of moving the arm
 * to a specified position.
 * The specified position can be changed via the moveArm() method.
 * 
 */
public class ArmSubsystem extends SubsystemBase {
  // lead motor
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;
  private final PIDController m_armPID;
  private final ArmFeedforward m_feedforward;
  private final DutyCycleEncoder m_encoder;

  public ArmSubsystem() {
    m_feedforward = new ArmFeedforward(0, Constants.Arm.kFeedforwardG, Constants.Arm.kFeedforwardV);
    m_leftMotor = new CANSparkMax(Constants.CanBus.leftPivotMotorID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.CanBus.rightPivotMotorID, MotorType.kBrushless);
    m_encoder = new DutyCycleEncoder(Constants.RioPorts.kArmAbsoluteEncoder);
    m_armPID = new PIDController(Constants.Arm.KPArmPivot, Constants.Arm.KIArmPivot, Constants.Arm.KDArmPivot);
  }
   
  
  
  @Override
  public void periodic() {
    
    }
  
}