
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
    private BooleanSupplier m_reset;

    private boolean amp;
    private boolean speaker;
    private boolean isShooting;

    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmWristSubsystem armwrist, LED led, 
        XboxController ops, BooleanSupplier trigger, BooleanSupplier amp, BooleanSupplier reset) {
        m_intakeAndShooter = intakeAndShooter;
        m_armwrist = armwrist;
        m_led = led;
        m_ops = ops;
        m_trigger = trigger;
        m_amp = amp;
        m_reset = reset;

        addRequirements(m_armwrist, m_led, m_intakeAndShooter);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        log("Is Shooter Ready", isShooting);
        // set isShooting to false when the note is not in the robot
        if (!m_intakeAndShooter.isIntakePhotogateTriggered() && !m_intakeAndShooter.isShooterPhotogateTriggered()) {
            isShooting = false;
        }
        // when the note is not in the robot: set arm and wrist to intake setpoint, start intake, stop shooter
        if (!m_intakeAndShooter.isLoaded() && !isShooting){
            amp = false; 
            speaker = false;
            m_armwrist.setWristSetpoint(Constants.Wrist.kIntakeSetpoint);
            if (m_armwrist.isWristAtSetpoint()) {
                m_armwrist.setArmSetpoint(Constants.Arm.kIntakeSetpoint); 
            }
            if (m_armwrist.isWristAtSetpoint() && m_armwrist.isArmAtSetpoint()) {
                m_intakeAndShooter.setIntakeSpeed(0.2);
            }
            if (!m_armwrist.isWristAtSetpoint() || !m_armwrist.isArmAtSetpoint()) {
                m_intakeAndShooter.intakeStop();
            }
            m_intakeAndShooter.shooterStop();
        }
        // roll back the note
        if ((m_intakeAndShooter.intakeSpeed() > 0) && m_intakeAndShooter.isShooterPhotogateTriggered() && !isShooting) {
            m_intakeAndShooter.setIntakeSpeed(-0.1);
        }
        // once the note is rolled back stop intake and set arm and wrist and start spinning shooter motors
        if ((m_intakeAndShooter.intakeSpeed() < 0) && m_intakeAndShooter.isLoaded() && !isShooting) {
            m_intakeAndShooter.intakeStop();
            m_armwrist.setArmSetpoint(Constants.Arm.kDriveSetpoint);
            m_armwrist.setWristSetpoint(Constants.Wrist.kDriveSetpoint);
            isShooting = true;
            speaker = true;
        }
        // if shooting for speaker have rpm at 1000
        if (isShooting && speaker) {
            m_intakeAndShooter.setShooterRPM(2000);
        }
        // if shooting for amp have rpm at 300
        if (isShooting && amp) {
            m_intakeAndShooter.setShooterRPM(1000);
        }
        // shoot if trigger is hit and the top and bottom shooter motors are close to the target rpm
        if (m_trigger.getAsBoolean() && m_intakeAndShooter.isReady()) {
            m_intakeAndShooter.setIntakeSpeed(0.5);
        }
        // if amp button is hit set arm and wrist to amp mode
        if (m_amp.getAsBoolean()) {
            m_intakeAndShooter.intakeStop();
            //m_intakeAndShooter.shooterStop();
            m_armwrist.setArmSetpoint(Constants.Arm.kAmpSetpoint);
            m_armwrist.setWristSetpoint(Constants.Wrist.kAmpSetpoint);
            amp = true;
        }
        // if reset button is hit set arm and wrist to intake mode
        if (m_reset.getAsBoolean()) {
            m_intakeAndShooter.intakeStop();
            //m_intakeAndShooter.shooterStop();
            m_armwrist.setWristSetpoint(Constants.Wrist.kIntakeSetpoint);
            m_armwrist.setArmSetpoint(Constants.Arm.kIntakeSetpoint);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
