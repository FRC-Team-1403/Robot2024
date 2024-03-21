// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import javax.swing.plaf.TreeUI;

import com.google.flatbuffers.Table;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team1403.robot.Constants.Operator;
import team1403.robot.Datables.Tables;
import team1403.robot.StateManager.GamePiece;
import team1403.robot.subsystems.arm.ArmStateGroup;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.SetpointArmCommand;
import team1403.robot.swerve.DefaultSwerveCommand;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.StateManager;

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
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;
  private final Tables m_dataTable;
  private DigitalInput m_intakePhotogate;
  private DigitalInput m_shooterPhotogate;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_swerve = new SwerveSubsystem();
    // m_arm = new ArmSubsystem();
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);
    m_dataTable = new Tables();
    // m_PhotonVisionCommand = new PhotonVisionCommand(m_limelight,m_swerve);
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

  // public Limelight getLimelight(){
  //   return m_limelight;
  // }

  public SwerveSubsystem getSwerveSubsystem() {
    return m_swerve;
  }

  public Tables getDataTables(){
    return m_dataTable;
  }

  public void configureOperatorInterface() {

    // Intake tipped towards cone
    // new Trigger(() -> xboxOperator.getAButton()).onFalse(
    // new SequentialCommandGroup(
    // new UpdateArmState(GamePiece.CONE_TOWARDS),
    // new InstantCommand(() ->
    // System.out.println(StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState())),
    // new SetpointArmCommand(m_arm,
    // StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
    // true)));

    m_operatorController.a().onTrue(
      new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CONE_TOWARDS))
        .andThen(
          new SetpointArmCommand(m_arm,
            () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                 false)
        ));
    
    //Intake Upright COne 
    m_operatorController.b().onTrue(
      new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CONE_UPRIGHT))
        .andThen(
          new SetpointArmCommand(m_arm,
            () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                true)));

   // Intake cube
   m_operatorController.x().onTrue(
      new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CUBE))
        .andThen(
          new SetpointArmCommand(m_arm,
            () -> StateManager.getInstance().getCurrentArmGroup().getFloorIntakeState(),
                true)));

    // // Shelf Intake
    m_operatorController.y().onTrue(
      new InstantCommand(() -> StateManager.getInstance().updateArmState(GamePiece.CONE_TOWARDS))
        .andThen(
          new SetpointArmCommand(m_arm,
            () -> StateManager.getInstance().getCurrentArmGroup().getSingleShelfIntakeState(),
                false)));

    // new Trigger(() -> xboxOperator.getPOV() == 180).onFalse(
    //     new SetpointArmCommand(m_arm, () -> ArmStateGroup.getTuck(), false));
    // new Trigger(() -> xboxOperator.getPOV() == 0).onFalse(
    //     new SetpointArmCommand(m_arm, () -> StateManager.getInstance().getCurrentArmGroup().getHighNodeState(), true));
    // new Trigger(() -> xboxOperator.getPOV() == 90).onFalse(
    //     new SetpointArmCommand(m_arm, () -> StateManager.getInstance().getCurrentArmGroup().getMiddleNodeState(),
    //         true));
    // new Trigger(() -> xboxOperator.getPOV() == 270).onFalse(
    //     new SetpointArmCommand(m_arm, () -> StateManager.getInstance().getCurrentArmGroup().getLowNodeState(), false));

  }

  // public void configureArmSetpoints() {

  //   XboxController xboxOperator = new XboxController(0);
    
  //   new Trigger(() -> xboxOperator.getAButton()).onTrue(
  //     new SequentialCommandGroup( 
  //       new InstantCommand( () -> m_arm.setAbsolutePivotAngle(89)), 
  //       new WaitCommand(1), 
  //       new InstantCommand( () -> m_arm.moveWrist(13)))); // high
  // new Trigger(() -> xboxOperator.getBButton()).onTrue(
  //   new SequentialCommandGroup( 
  //     new InstantCommand( () -> m_arm.setAbsolutePivotAngle(44)), 
  //     new WaitCommand(1), 
  //     new InstantCommand( () -> m_arm.moveWrist(10)))); // mid
  // new Trigger(() -> xboxOperator.getXButton()).onTrue(
  //   new SequentialCommandGroup( 
  //     new InstantCommand( () -> m_arm.setAbsolutePivotAngle()), 
  //     new WaitCommand(1), 
  //     new InstantCommand( () -> m_arm.moveWrist(5)))); // low

  // new Trigger(() -> xboxOperator.getYButton()).onTrue(
  //   new SequentialCommandGroup( 
  //     new InstantCommand( () -> m_arm.setAbsolutePivotAngle(0)), 
  //     new WaitCommand(1), 
  //     new InstantCommand( () -> m_arm.moveWrist(0)))); // tuck


  // }

}
