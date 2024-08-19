// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team1403.robot.commands.IntakeShooterLoop;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private IntakeShooterLoop m_combinedCommand;

  public Robot() {
    super(Constants.kLoopTime);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    Logger.recordMetadata("Team 1403", "2024 Robot"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }
    else {
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    m_robotContainer = new RobotContainer();
    m_combinedCommand =  new IntakeShooterLoop(
      m_robotContainer.getIntakeShooterSubsystem(), m_robotContainer.getArmSubsystem(), 
      m_robotContainer.getWristSubsystem(), m_robotContainer.getLEDSubsystem(), m_robotContainer.getOps(),
      () -> m_robotContainer.getOps().getRightTriggerAxis() >= 0.5, // shoot
      () -> m_robotContainer.getOps().getBButton(), // amp
      () -> m_robotContainer.getOps().getXButton(), // loading station
      () -> m_robotContainer.getOps().getAButton(), // reset to intake
      () -> m_robotContainer.getOps().getLeftTriggerAxis() >= 0.5, // stage line shot
      () -> m_robotContainer.getOps().getPOV() == 0, // center line shot
      () -> m_robotContainer.getOps().getYButton(), // reset to netural
      () -> m_robotContainer.getOps().getLeftBumper(), // launchpad
      () -> m_robotContainer.getOps().getLeftY(), // expel
      () -> m_robotContainer.getOps().getRightBumper(), // amp shooting
      () -> m_robotContainer.getOps().getPOV() == 90); // feeding
    // intake out joystick left up  v 
    // AutoSelector.initAutoChooser();

    // SmartDashboard.putNumber("Servo Angle", 180);
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Mat image = new Mat();
    // long ret = CameraServer.getVideo().grabFrame(image);

    // if(ret != 0)
    // {

    //   Imgproc.rectangle(image, getCenteredRect(image, 30, 20), new Scalar(0,255,0), 2, 0);
    // }
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("auto name: " + m_autonomousCommand.getName());
    // schedule the autonomous command (example)
    m_combinedCommand.cancel();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { //100
    // m_robotContainer.getArmSubsystem().moveArm(Constants.Arm.kIntakeSetpoint);
    // m_robotContainer.getWristSubsystem().setWristAngle(Constants.Wrist.kIntakeSetpoint);

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.getLimelight().setDefaultCommand(m_VisionCommand);
    m_robotContainer.getSwerveSubsystem().setEnableRotDriftCorrect(true);
    m_combinedCommand.schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // m_robotContainer.getHangerSubsystem().runHanger(0.1);
    // m_robotContainer.getHangerSubsystem().setServoAngle(SmartDashboard.getNumber("Servo Angle", 180));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
