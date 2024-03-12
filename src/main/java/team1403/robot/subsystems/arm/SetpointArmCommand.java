package team1403.robot.subsystems.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * Creates the arm set point class.
 */
public class SetpointArmCommand extends Command {
  private ArmState m_state;
  private TrapezoidProfile m_pivotProfile;
  private final ArmSubsystem m_arm;

  private double m_startTime;

  private final Supplier<ArmState> m_armStateSupplier;

  private boolean m_ignoreLimit;
  
  /**
   * Initializes the class.
   */
  public SetpointArmCommand(ArmSubsystem arm, Supplier<ArmState> armStateSupplier, boolean ignoreLimit) {
    this.m_arm = arm;
    this.m_armStateSupplier = armStateSupplier;
    this.m_ignoreLimit = ignoreLimit;

    addRequirements(arm);
  }
  
  @Override
  public void initialize() {
    m_state = m_armStateSupplier.get();
    m_pivotProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(360, 165), //high --> 360, 165 //slow --> 20, 10
      new TrapezoidProfile.State(m_state.armPivot, 1),
      new TrapezoidProfile.State(m_arm.getAbsolutePivotAngle(), 0));
    this.m_startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double deltaT = Timer.getFPGATimestamp() - m_startTime;
    double pivotPosition = m_pivotProfile.calculate(deltaT).position;
    m_arm.ignoreExtensionLimit(m_ignoreLimit);
    m_arm.moveArm(m_state.intakeSpeed, pivotPosition, m_state.armLength);
    m_arm.moveWrist(m_state.wristAngle);

    super.execute();
  }

  @Override
  public boolean isFinished() {
    m_arm.ignoreExtensionLimit(false);
    return m_pivotProfile.isFinished(Timer.getFPGATimestamp() - m_startTime);
  }
}
