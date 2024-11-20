
package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.lib.util.CougarLogged;
import team1403.robot.Constants;
import team1403.robot.Constants.Setpoints;
import team1403.robot.subsystems.ArmWristSubsystem;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.LED;

public class IntakeShooterLoop extends Command implements CougarLogged {
    private IntakeAndShooter m_intakeAndShooter;
    private ArmWristSubsystem m_armwrist;
    private LED m_led;
    private XboxController m_ops;
    private BooleanSupplier m_trigger;
    private BooleanSupplier m_amp;
    private boolean isShooting;

    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmWristSubsystem armwrist, LED led, 
        XboxController ops, BooleanSupplier trigger, BooleanSupplier amp) {
        m_intakeAndShooter = intakeAndShooter;
        m_armwrist = armwrist;
        m_led = led;
        m_ops = ops;
        m_trigger = trigger;
        m_amp = amp;

        addRequirements(m_armwrist, m_led, m_intakeAndShooter);
    }

    @Override
    public void initialize() {
        Blackbox.requestedSetpoint = Setpoints.kDriveSetpoint;
    }

    // private void applySetpoint(SonicBlasterSetpoint setpoint) {
    //     m_intakeAndShooter.applySetpoint(setpoint);
    //     m_armwrist.applySetpoint(setpoint);
    // }
    
    @Override
    public void execute() {
        if (!m_intakeAndShooter.isIntakePhotogateTriggered() && !m_intakeAndShooter.isShooterPhotogateTriggered()) {
            isShooting = false;
        }
        if (!Constants.IntakeAndShooter.isLoaded && !isShooting){
            m_armwrist.setArmSetpoint(Constants.Arm.kIntakeSetpoint);
            m_armwrist.setWristSetpoint(Constants.Wrist.kIntakeSetpoint);
            m_intakeAndShooter.setIntakeSpeed(0.2);
            m_intakeAndShooter.shooterStop();
        }
        if (m_intakeAndShooter.isShooterPhotogateTriggered() && !isShooting) {
            m_intakeAndShooter.setIntakeSpeed(-0.1);
        }
        if (m_trigger.getAsBoolean() && Constants.IntakeAndShooter.isLoaded) {
            isShooting = true;
            m_intakeAndShooter.setShooterRPM(1000);
        }
        if (Constants.IntakeAndShooter.isLoaded && !isShooting) {
            m_intakeAndShooter.intakeStop();
            m_armwrist.setArmSetpoint(Constants.Arm.kDriveSetpoint);
            m_armwrist.setWristSetpoint(Constants.Wrist.kDriveSetpoint);
        }
        if (m_intakeAndShooter.isReady()) {
            m_intakeAndShooter.setIntakeSpeed(0.5);
        }
        if (m_amp.getAsBoolean()) {
            m_armwrist.setArmSetpoint(Constants.Arm.kAmpSetpoint);
            m_armwrist.setWristSetpoint(Constants.Wrist.kAmpSetpoint);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
