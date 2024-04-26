// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team1403.robot.commands.AutoIntakeShooterLoop;
import team1403.robot.commands.IntakeShooterLoop;
import team1403.robot.commands.TriggerShotCommand;
import team1403.robot.subsystems.HangerSubsystem;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.LED;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;
import team1403.robot.swerve.DefaultSwerveCommand;
import team1403.robot.swerve.Limelight;
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
  private HangerSubsystem m_hanger;
  private LED m_led;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;

  private SendableChooser<String> autoChooser;

  private IntakeShooterLoop m_combinedCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_limelight = new Limelight();
    m_swerve = new SwerveSubsystem(m_limelight);
    m_arm = new ArmSubsystem();
    m_wrist = new Wrist(m_arm);
    m_endeff = new IntakeAndShooter();
    m_led = new LED();
    m_hanger = new HangerSubsystem();
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);
    // Enables power distribution logging
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);

    m_combinedCommand = new IntakeShooterLoop(
      getIntakeShooterSubsystem(), getArmSubsystem(), 
      getWristSubsystem(), getLEDSubsystem(), getOps(),
      () -> getOps().getRightTriggerAxis() > 0.4, // shoot
      () -> getOps().getHID().getBButton(), // amp
      () -> getOps().getHID().getXButton(), // loading station
      () -> getOps().getHID().getAButton(), // reset to intake
      () -> getOps().getLeftTriggerAxis() > 0.4, // stage line shot
      () -> getOps().povUp().getAsBoolean(), // center line shot
      () -> getOps().getHID().getYButton(), // reset to netural
      () -> getOps().getHID().getLeftBumper(), // launchpad
      () -> getOps().getLeftY(), // expel
      () -> getOps().getHID().getRightBumper() // amp shooting
      );

    // NamedCommands.registerCommand("stop", new InstantCommand(() -> m_swerve.stop()));
    // NamedCommands.registerCommand("First Piece", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> false, () -> false, false, () -> false, false));
    // NamedCommands.registerCommand("Shoot Side", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> true, () -> false, true, () -> false, false));
    // NamedCommands.registerCommand("Shoot", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> true, () -> false, false, () -> false, false));
    // NamedCommands.registerCommand("Reset Shooter", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> false, () -> false, false, () -> false, false));
    // NamedCommands.registerCommand("First Piece Side",  new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> false, () -> false, true, () -> true, false));
    // NamedCommands.registerCommand("Second Source Shoot", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> true, () -> false, false, () -> false, true));
    // // NamedCommands.registerCommand("IntakeClose", new IntakeCommand(m_endeff, m_arm, m_wrist,  Constants.Arm.kDriveSetpoint, Constants.Wrist.kDriveSetpoint, Constants.IntakeAndShooter.kCloseRPM));    
    // // NamedCommands.registerCommand("ShootLoaded", new ShootCommand(m_endeff, m_arm, m_wrist));
    // NamedCommands.registerCommand("Trigger Shot", new TriggerShotCommand(m_endeff, m_wrist));

    autoChooser = new SendableChooser<>();

    autoChooser.addOption("Example Auto", "NewPath");
    autoChooser.setDefaultOption("None", "NoAuto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

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
    Pose2d ampLocationPose2d = new Pose2d(new Translation2d(Units.inchesToMeters(72.5), Units.inchesToMeters(303.0)),
                Rotation2d.fromDegrees(90.0));
    
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(
        m_swerve,
        () -> -deadband(m_driverController.getLeftX(), 0.01),
        () -> -deadband(m_driverController.getLeftY(), 0.01),
        () -> -deadband(m_driverController.getRightX(), 0.01),
        () -> m_driverController.getHID().getYButton(),
        () -> m_driverController.getHID().getXButton(),
        () -> m_driverController.getHID().getAButton(),
        () -> -0, // AMP Location 
        () -> 0,
        // blue 
        // () -> 0.42,
        // () -> 5.53,
        () -> m_driverController.getRightTriggerAxis(),
        () -> m_driverController.getLeftTriggerAxis() > .5));


    m_driverController.b().onTrue(new InstantCommand(() -> m_swerve.zeroGyroscope(), m_swerve));

    m_operatorController.povLeft().onTrue(
      new InstantCommand(() -> m_hanger.runHanger(1), m_hanger).andThen(() -> m_led.setLedColor(-0.91)));
      
    m_operatorController.povDown().onTrue(
      new InstantCommand(() -> m_hanger.runHanger(-1), m_hanger).andThen(() -> m_led.setLedColor(-.99)));
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

  private ChoreoTrajectory traj;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    traj = Choreo.getTrajectory(autoChooser.getSelected());

    if(traj == null)
    {
      return Commands.none();
    }

    BooleanSupplier redSupplier = () -> DriverStation.getAlliance().isPresent() && 
    DriverStation.getAlliance().get() == Alliance.Red;

    Command auto = Choreo.choreoSwerveCommand(traj, () -> m_swerve.getPose(), 
    new PIDController(Constants.Swerve.kPCTranslation, Constants.Swerve.kICTranslation, Constants.Swerve.kDCTranslation), 
    new PIDController(Constants.Swerve.kPCTranslation, Constants.Swerve.kICTranslation, Constants.Swerve.kDCTranslation), 
    new PIDController(Constants.Swerve.kPCAutoTurning, Constants.Swerve.kICAutoTurning, Constants.Swerve.kDCAutoTurning), 
    (speed) -> m_swerve.driveNoOffset(speed), 
    redSupplier, m_swerve);

    Pose2d initialPose = redSupplier.getAsBoolean() ? 
    traj.getFlippedInitialPose() : traj.getInitialPose();

    if(redSupplier.getAsBoolean())
      Logger.recordOutput("Odometery/ChoreoTrajectory", traj.flipped().getPoses());
    else
      Logger.recordOutput("Odometery/ChoreoTrajectory", traj.getPoses());

    

    return Commands.sequence(
      Commands.runOnce(() -> m_swerve.resetOdometry(initialPose), m_swerve), 
      auto, 
      new InstantCommand(() -> m_swerve.stop(), m_swerve));
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

  public HangerSubsystem getHangerSubsystem(){
    return m_hanger;
  }

  public LED getLEDSubsystem() {
    return m_led;
  }

  public IntakeShooterLoop getCombinedCommand() {
    return m_combinedCommand;
  }
}
