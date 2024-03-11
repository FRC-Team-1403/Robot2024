package team1403.robot.subsystems.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Moves the arm to a given position while avoiding any obstancles.
 */
public class SequentialMoveArmCommand extends CommandBase {
  private final ArmSubsystem m_arm;

  private double m_intialPivotAngle;
  private double m_intialExtensionLength;
  private double m_initialWristAngle;
  private double m_initialIntakeSpeed;

  private final Supplier<ArmState> m_endState;
  private ArmState m_firstState;

  private TrapezoidProfile m_pivotProfile;

  private double m_startTime;

  private boolean m_isFinished = false;

  private boolean m_ignoreLimit;


  public SequentialMoveArmCommand(ArmSubsystem arm, Supplier<ArmState> endState, boolean ignoreLimit) {
    this.m_endState = endState;
    this.m_arm = arm;
    this.m_ignoreLimit = ignoreLimit;
  }

  @Override
  public void initialize() {
    this.m_intialPivotAngle = m_arm.getPivotAngleSetpoint();
    this.m_intialExtensionLength = m_arm.getExtensionLengthSetpoint();
    this.m_initialWristAngle = m_arm.getAbsoluteWristAngle();
    this.m_initialIntakeSpeed = m_arm.getIntakeSpeedSetpoint();
    
    this.m_pivotProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(360, 165), //high --> 360, 165 //slow --> 20, 10
      new TrapezoidProfile.State(m_endState.get().armPivot, 1),
      new TrapezoidProfile.State(m_intialPivotAngle, 0));

    this.m_startTime = Timer.getFPGATimestamp();

    this.m_firstState = new ArmState(m_intialExtensionLength , m_initialWristAngle, m_endState.get().armPivot, m_initialIntakeSpeed);

    // super.initialize();
  }


  @Override
  public void execute() {
    double deltaT = Timer.getFPGATimestamp() - m_startTime;
    if(!m_pivotProfile.isFinished(deltaT)) {
      double pivotPosition = m_pivotProfile.calculate(deltaT).position;
      m_arm.ignoreExtensionLimit(m_ignoreLimit);
      m_arm.moveArm(this.m_firstState.intakeSpeed, pivotPosition, this.m_firstState.armLength);
      m_arm.moveWrist(this.m_firstState.wristAngle);
      
    } else {
      m_arm.ignoreExtensionLimit(m_ignoreLimit);
      m_arm.moveArm(this.m_endState.get());
      m_isFinished = true;
    }
    // super.execute();
  }

  @Override
  public boolean isFinished() {
    return m_isFinished && m_arm.isAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("asdfasdfasdf");
  }
}
