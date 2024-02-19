// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import com.google.flatbuffers.Table;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team1403.robot.Datables.Tables;
import team1403.robot.commands.ArmCommand;
import team1403.robot.commands.RunWrist;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;
import team1403.robot.swerve.DefaultSwerveCommand;
import team1403.robot.swerve.Limelight;
import team1403.robot.swerve.PhotonVisionCommand;
import team1403.robot.swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private SwerveSubsystem m_swerve;
  private Limelight m_limelight;
  //private AimbotCommand m_aimbot;
  private ArmSubsystem m_arm;
  private Wrist m_wrist;
  private IntakeAndShooter m_endeff;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;
  private final PhotonVisionCommand m_PhotonVisionCommand; 

  private final PowerDistribution m_powerDistribution;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_limelight = new Limelight();
    m_swerve = new SwerveSubsystem(m_limelight);
    m_arm = new ArmSubsystem();
    m_wrist = new Wrist(m_arm);
    m_endeff = new IntakeAndShooter();
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);
    m_PhotonVisionCommand = new PhotonVisionCommand(m_limelight,m_swerve);
    // Enables power distribution logging
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);
    NamedCommands.registerCommand("stop", new InstantCommand(() -> m_swerve.stop()));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // Setting default command of swerve subPsystem

    m_swerve.setDefaultCommand(new DefaultSwerveCommand(
        m_swerve,
        () -> -deadband(m_driverController.getLeftX(), 0),
        () -> -deadband(m_driverController.getLeftY(), 0),
        () -> -deadband(m_driverController.getRightX(), 0),
        () -> m_driverController.y().getAsBoolean(),
        () -> m_driverController.x().getAsBoolean(),
        () -> m_driverController.a().getAsBoolean(),
        () -> 12.70, //3 significant digits
        () -> 5.36,
        () -> m_driverController.getRightTriggerAxis()));
    
    m_driverController.b().onTrue(new InstantCommand(() -> m_swerve.zeroGyroscope(), m_swerve)); 
    //m_operatorController.a().onTrue(m_aimbot);
    m_operatorController.a().onTrue(new InstantCommand(() -> m_arm.moveArm(130)));
    m_operatorController.x().onTrue(new SequentialCommandGroup(
    new RunWrist(m_wrist,140),
    new ArmCommand(m_arm,Constants.Arm.kIntakeSetpoint,2),
    new RunWrist(m_wrist,133)
    ));

    m_operatorController.y().onTrue(new RunWrist(m_wrist,179));
    m_operatorController.b().onTrue(new ArmCommand(m_arm,220,2));
    // m_operatorController.y().onTrue(new SequentialCommandGroup(
    // new RunWrist(m_wrist,307),
    // new ArmCommand(m_arm,205,2)
    // ));
  }
  
  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutoSelector.getSelectedAuto().andThen(new InstantCommand(() -> m_swerve.stop()));
  }

  public Limelight getLimelight(){
    return m_limelight;
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return m_swerve;
  }

  public CommandXboxController getOps() {
    return m_operatorController;
  }

  public CommandXboxController getDriverController() {
    return m_driverController;
  }

  public ArmSubsystem getArmSubsystem() {
    return m_arm;
  }

  public Wrist getWristSubsystem() {
    return m_wrist;
  }

  public IntakeAndShooter getIntakeShooterSubsystem() {
    return m_endeff;
  }

}
