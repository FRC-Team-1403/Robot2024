
package team1403.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Logged;
import team1403.lib.util.CougarLogged;
import team1403.robot.Constants;
import team1403.robot.Constants.Setpoints;
import team1403.robot.subsystems.ArmWristSubsystem;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.LED;
import team1403.robot.subsystems.SonicBlasterSetpoint;
import team1403.robot.subsystems.LED.LEDState;

public class IntakeShooterLoop extends Command implements CougarLogged {
    private IntakeAndShooter m_intakeAndShooter;
    private ArmWristSubsystem m_armwrist;
    private LED m_led;
    private XboxController m_ops;
    private BooleanSupplier m_trigger;

    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmWristSubsystem armwrist, LED led, 
        XboxController ops, BooleanSupplier trigger) {
        m_intakeAndShooter = intakeAndShooter;
        m_armwrist = armwrist;
        m_led = led;
        m_ops = ops;
        m_trigger = trigger;

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
        if (m_trigger.getAsBoolean() && Constants.IntakeAndShooter.isLoaded) {
            m_intakeAndShooter.setShooterRPM(1000);
        }
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
