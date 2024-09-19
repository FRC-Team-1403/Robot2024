// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team1403.lib.util.AutoUtil;
import team1403.robot.commands.AutoIntakeShooterLoop;
import team1403.robot.commands.TriggerShotCommand;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.HangerSubsystem;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.LED;
import team1403.robot.subsystems.LED.LEDState;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;
import team1403.robot.swerve.DefaultSwerveCommand;
import team1403.robot.swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private SwerveSubsystem m_swerve;
  //private AimbotCommand m_aimbot;
  private ArmSubsystem m_arm;
  private Wrist m_wrist;
  private IntakeAndShooter m_endeff;
  private HangerSubsystem m_hanger;
  private LED m_led;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;

  private SendableChooser<Command> autoChooser;
  private Command m_pathFinder = Commands.none();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    m_swerve = new SwerveSubsystem();
    m_arm = new ArmSubsystem();
    m_wrist = new Wrist(m_arm);
    m_endeff = new IntakeAndShooter();
    m_led = new LED();
    m_hanger = new HangerSubsystem();
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);
    // Enables power distribution logging
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);

    NamedCommands.registerCommand("stop", new InstantCommand(() -> m_swerve.stop()));
    NamedCommands.registerCommand("First Piece", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> false, () -> false, false, () -> false, false));
    NamedCommands.registerCommand("Shoot Side", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> true, () -> false, true, () -> false, false));
    NamedCommands.registerCommand("Shoot", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> true, () -> false, false, () -> false, false));
    NamedCommands.registerCommand("Reset Shooter", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> false, () -> false, false, () -> false, false));
    NamedCommands.registerCommand("First Piece Side",  new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> false, () -> false, true, () -> true, false));
    NamedCommands.registerCommand("Second Source Shoot", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> true, () -> false, false, () -> false, true));
    // NamedCommands.registerCommand("IntakeClose", new IntakeCommand(m_endeff, m_arm, m_wrist,  Constants.Arm.kDriveSetpoint, Constants.Wrist.kDriveSetpoint, Constants.IntakeAndShooter.kCloseRPM));    
    // NamedCommands.registerCommand("ShootLoaded", new ShootCommand(m_endeff, m_arm, m_wrist));
    NamedCommands.registerCommand("Trigger Shot", new TriggerShotCommand(m_endeff, m_wrist));

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Choreo Auto", AutoUtil.loadChoreoAuto("test", m_swerve));

    Constants.kDriverTab.add("Auto Chooser", autoChooser);

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
    // red

    Translation2d pos_blue = new Translation2d(-0.038099999999999995,  5.547867999999999);
    Translation2d pos_red = new Translation2d(16.579342,  5.547867999999999);
    Pose2d pos_blue_shoot = new Pose2d(new Translation2d(1.5, 5.4), new Rotation2d());
    Pose2d pos_red_shoot = new Pose2d(new Translation2d(15, 5.4), new Rotation2d(Math.PI));
    
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(
        m_swerve,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.05),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.05),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), 0.05),
        () -> m_driverController.getHID().getYButtonPressed(),
        () -> m_driverController.getHID().getXButton(),
        () -> m_driverController.getHID().getAButton(),
        () -> m_driverController.getHID().getLeftBumper(),
        () -> m_driverController.getHID().getRightBumper(),
        () -> Constants.kAllianceSupplier.get() == Alliance.Blue ? pos_blue : pos_red,
        () -> m_driverController.getRightTriggerAxis(),
        () -> m_driverController.getLeftTriggerAxis()));


    m_driverController.b().onTrue(new InstantCommand(() -> m_swerve.zeroHeading(), m_swerve));

    m_driverController.rightBumper().onTrue(Commands.runOnce(() -> {
      Pose2d tar = pos_red_shoot;
      if(Constants.kAllianceSupplier.get() == Alliance.Blue) tar = pos_blue_shoot;
      Blackbox.targetPosition = tar;
      m_pathFinder = AutoUtil.pathFindToPose(tar);
    }).andThen(Commands.deferredProxy(() -> m_pathFinder)));

    m_driverController.leftStick()
      .or(() -> Math.hypot(m_driverController.getLeftX(), m_driverController.getLeftY()) > 0.2)
        .onTrue(Commands.runOnce(() -> m_pathFinder.cancel()));

    m_operatorController.povLeft().onTrue(
      new InstantCommand(() -> m_hanger.runHanger(1), m_hanger));
      
    m_operatorController.povDown().onTrue(
      new InstantCommand(() -> m_hanger.runHanger(-1), m_hanger));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected().andThen(() -> m_swerve.stop(), m_swerve);
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return m_swerve;
  }

  public XboxController getOps() {
    return m_operatorController.getHID();
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

  public HangerSubsystem getHangerSubsystem(){
    return m_hanger;
  }

  public LED getLEDSubsystem() {
    return m_led;
  }
}
