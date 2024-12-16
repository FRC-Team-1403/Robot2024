
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
    private BooleanSupplier m_trigger;
    private BooleanSupplier m_loading;
    private double m_fpga;
    private LED m_led;
    private BooleanSupplier m_resetToIntake;
    private DoubleSupplier m_expel;
    private XboxController m_ops;
    private int m_counter;

}
