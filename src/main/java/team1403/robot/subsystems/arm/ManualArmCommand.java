
package team1403.robot.subsystems.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * class ArmCommands is the where the commands for Arm.java is located
 */
public class ManualArmCommand extends CommandBase {
  private final DoubleSupplier m_armAngleSupplier;
  private final DoubleSupplier m_wristAngleSupplier;
  private final DoubleSupplier m_armExtensionDecreaseSupplier;
  private final DoubleSupplier m_armExtensionIncreaseSupplier;
  private final BooleanSupplier m_wheelIntakeSupplier;
  private final BooleanSupplier m_wheelOuttakeSupplier;


  private final ArmSubsystem m_arm;
  /**
   * Defines the constructor for ArmCommands,
   * sets the instant class level vaiables,
   * to new variables.
   *
   * @param arm the Armsubsystem
   * @param armAngle function that determines arm angle, -1 to 1
   * @param wristAngle function that determines wrist angle, -1 to 1
   * @param armExtensionDecrease function that determines
    the increase of arm extension, 0 to 1
    the decrease of arm extension, 0 to 1
   */
  public ManualArmCommand(ArmSubsystem arm, DoubleSupplier armAngle, DoubleSupplier wristAngle,
      DoubleSupplier armExtensionDecrease, DoubleSupplier armExtensionIncrease,
        BooleanSupplier wheelIntake, BooleanSupplier wheelOuttake) {
    this.m_wheelIntakeSupplier = wheelIntake;
    this.m_arm = arm;
    this.m_armAngleSupplier = armAngle;
    this.m_wristAngleSupplier = wristAngle;
    this.m_armExtensionDecreaseSupplier = armExtensionDecrease;
    this.m_armExtensionIncreaseSupplier = armExtensionIncrease;
    this.m_wheelOuttakeSupplier = wheelOuttake;

    addRequirements(arm);
  }
  // for the driver to remove the cube, for testing purposes may be used if the driver wants it
  public void driverOutManualOuttake() {
    double wristAngle = m_arm.getWristAngleSetpoint();
    wristAngle += m_wristAngleSupplier.getAsDouble() * 4;
    wristAngle = m_arm.limitWristAngle(wristAngle);
    double pivotAngle = m_arm.getPivotAngleSetpoint();
    pivotAngle += (-1 * m_armAngleSupplier.getAsDouble());
    pivotAngle = m_arm.limitPivotAngle(pivotAngle);
    m_arm.moveArm(wristAngle, 1, pivotAngle, m_arm.getExtensionLengthSetpoint());
  }

  @Override
  public void execute() {
    m_arm.ignoreExtensionLimit(true);
    double pivotAngle = m_arm.getPivotAngleSetpoint();
    double wristAngle = m_arm.getWristAngleSetpoint();
    double armExtension = m_arm.getExtensionLengthSetpoint();

    wristAngle += m_wristAngleSupplier.getAsDouble() * 4;
    wristAngle = m_arm.limitWristAngle(wristAngle);

    pivotAngle += (-1 * m_armAngleSupplier.getAsDouble());
    pivotAngle = m_arm.limitPivotAngle(pivotAngle);

    if (m_armExtensionDecreaseSupplier.getAsDouble() > 0) {
      armExtension -= m_armExtensionDecreaseSupplier.getAsDouble() / 2;
    } else if (m_armExtensionIncreaseSupplier.getAsDouble() > 0) {
      armExtension += m_armExtensionIncreaseSupplier.getAsDouble() / 2;
    }

    armExtension = m_arm.limitExtensionLength(armExtension);

    SmartDashboard.putNumber("Arm Extension Setpoint", armExtension);

    if (this.m_wheelIntakeSupplier.getAsBoolean()) {
      m_arm.moveArm(wristAngle, -1, pivotAngle, armExtension);
    } else if (this.m_wheelOuttakeSupplier.getAsBoolean()) {
      m_arm.moveArm(wristAngle, 1, pivotAngle, armExtension);
    } else {
      m_arm.moveArm(wristAngle, 0, pivotAngle, armExtension);
    }

  }

}
