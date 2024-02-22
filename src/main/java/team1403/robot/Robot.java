// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team1403.robot.Datables.ShooterValues;
import team1403.robot.swerve.PhotonVisionCommand;

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
  private PhotonVisionCommand m_VisionCommand;
  private RobotContainer m_robotContainer;
  private double tempWristAngle;
  private double tempArmAngle;

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

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }
    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.

    m_robotContainer = new RobotContainer();
    m_VisionCommand = new PhotonVisionCommand(m_robotContainer.getLimelight(),m_robotContainer.getSwerveSubsystem());

    AutoSelector.initAutoChooser();

    // tempWristAngle = 100;
    // tempArmAngle = 130;
    m_robotContainer.getSwerveSubsystem().zeroGyroscope();
    m_robotContainer.getSwerveSubsystem().getOdometer().setPose(new Pose2d( new Translation2d(1.39,5.52), new Rotation2d(0)));
    m_robotContainer.getSwerveSubsystem().getNavxAhrs().zeroYaw();
    SmartDashboard.putNumber("Shooting Setpoint", Constants.IntakeAndShooter.kShootingAngle);
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

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { //100
    m_robotContainer.getArmSubsystem().moveArm(Constants.Arm.kIntakeSetpoint);
    m_robotContainer.getWristSubsystem().setWristAngle(Constants.Wrist.kIntakeSetpoint);

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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Constants.IntakeAndShooter.kShootingAngle = SmartDashboard.getNumber("Shooting Setpoint",0);

    // for testing only
    // m_robotContainer.getWristSubsystem().setWristAngle(200); //135
    // m_robotContainer.getWristSubsystem().setWristAngle(tempWristAngle); //100
    // m_robotContainer.getArmSubsystem().moveArm(tempArmAngle);
    // m_robotContainer.getArmSubsystem().setArmSpeed(m_robotContainer.getOps().getRightY() * -0.3);
    // m_robotContainer.getWristSubsystem().increaseWristAngle(m_robotContainer.getOps().getLeftY());

    // m_robotContainer.getIntakeShooterSubsystem().setIntakeSpeed(0.3);      

    if(m_robotContainer.getOps().leftBumper().getAsBoolean())
    {
      m_robotContainer.getIntakeShooterSubsystem().setIntakeSpeed(1);
    }
    else if(m_robotContainer.getOps().rightBumper().getAsBoolean()){ 
      m_robotContainer.getIntakeShooterSubsystem().setIntakeSpeed(-.075);
    } else if(m_robotContainer.getOps().povUp().getAsBoolean())
      m_robotContainer.getIntakeShooterSubsystem().setShooterSpeed(1);
    else{
      m_robotContainer.getIntakeShooterSubsystem().setIntakeSpeed(0.0);
      m_robotContainer.getIntakeShooterSubsystem().setShooterSpeed(0.0);
    }
    
    // m_robotContainer.getArmSubsystem().moveArm(Constants.Arm.kIntakeSetpoint);
    // m_robotContainer.getWristSubsystem().setWristAngle(Constants.Wrist.kIntakeSetpoint);

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
