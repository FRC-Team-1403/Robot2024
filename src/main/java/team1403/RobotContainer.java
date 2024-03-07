// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team1403.Constants.Intake;
// import team1403.commands.IntakeCommand;
// import team1403.commands.ShootCommand;
import team1403.subsystems.IntakeSubsystem;
// import team1403.commands.IntakeRollback;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private IntakeSubsystem m_intake;
  private boolean isIntake = false;
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);

   
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_intake = new IntakeSubsystem();
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factoofdszzzzzzzsssssssssssssssssssssssx  tries in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // new Trigger(() -> SmartDashboard.getBoolean("button y", false))
    //   .onTrue(new IntakeRollback(m_intake));
    // new Trigger(() -> SmartDashboard.getBoolean("button a",false))
    //   .onTrue(new IntakeCommand(m_intake, 1.0));
    // new Trigger(() -> SmartDashboard.getBoolean("button b",false))
    //   .onTrue(new ShootCommand(m_intake));
    // new Trigger(() -> SmartDashboard.getBoolean("button x",false))
    //   // .onTrue(new ShooterReady(m_intake, 1000));
    //         .onTrue(new InstantCommand(() -> m_intake.setShooterSpeed(.1)));

    // new Trigger(() -> SmartDashboard.getBoolean("button test",false))
    //   .onTrue(new InstantCommand(() -> m_intake.setShooterSpeed(.1)));

    m_driverController.y().onTrue(new InstantCommand(() -> m_intake.setIntakeSpeed(-0.1)));
    m_driverController.a().onTrue(new InstantCommand(() -> m_intake.setIntakeSpeed(0.4)));
    m_driverController.b().onTrue(new InstantCommand(() -> m_intake.setShooterSpeed(1)));
    m_driverController.x().whileTrue(new RunCommand(() -> m_intake.setShooterSpeed(0)));
    m_driverController.leftTrigger().onTrue(new InstantCommand(() -> m_intake.stopAll()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
 // public Command getAutonomousCommand() {}
}
