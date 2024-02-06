// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team1403.commands.IntakeCommand;
import team1403.subsystems.IntakeSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private IntakeSubsystem m_intake = new IntakeSubsystem();
  private IntakeCommand m_intakeCommand = new IntakeCommand(m_intake);
  private CommandXboxController m_controller = new CommandXboxController(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putNumber("shooter", 0);
    SmartDashboard.putNumber("intake", 0);
    SmartDashboard.putBoolean("Second LimitSwitch", false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    ////m_intakeCommand.schedule();
    m_intake.setShooterRpm(0.0);
    m_intake.setIntakeRpm(0.0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double m_topNeoSpeed = SmartDashboard.getNumber("shooter", 0);
    //double m_topNeoSpeed = m_controller.getRightTriggerAxis();
    // if(m_controller.b().getAsBoolean())
    // {
    //   m_intake.setShooterRpm(m_topNeoSpeed);
    // }
    // else
    // {
    //   m_intake.setShooterRpm(0.0);
    // }

    
    // if(m_controller.a().getAsBoolean())
    //   m_intake.setIntakeSpeed(1);
    // else if(m_controller.y().getAsBoolean())
    //   m_intake.setIntakeRpm(-600);
    // else
    //   m_intake.setIntakeRpm(0.0);
    boolean secondLimitSwitch = SmartDashboard.getBoolean("Second LimitSwitch", false);
    if (m_controller.b().getAsBoolean()) {
      while (!secondLimitSwitch) {
        m_intake.setIntakeRpm(5000);
      }
      m_intake.setIntakeRpm(0);
      while (secondLimitSwitch) {
        m_intake.setIntakeSpeed(-0.1);
      }
      m_intake.setIntakeSpeed(0);
      m_intake.setShooterRpm(5000);
      try {
        wait(3000);
      } catch (Exception e) {
        e.printStackTrace();
      }
      m_intake.setIntakeRpm(5000);
      try {
        wait(1000);
      } catch (Exception e) {
        e.printStackTrace();
      }
      m_intake.setIntakeRpm(0);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
