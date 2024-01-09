// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot.swerve;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class SwerveControllerCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final HolonomicDriveController m_controller;
  private final Supplier<Rotation2d> m_desiredRotation;
  private final SwerveSubsystem m_subsystem;
  private PIDController m_xController;
  private PIDController m_yController;
  private ProfiledPIDController m_thetaController;

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path.
   * This is left to the user to do since it is not appropriate for paths with nonstationary
   * endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
   *     time step.
   * @param subsystem The subsystems to require.
   */
  public SwerveControllerCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      SwerveSubsystem subsystem) {
    this(
        trajectory,
        pose,
        new HolonomicDriveController(
            requireNonNullParam(xController, "xController", "SwerveControllerCommand"),
            requireNonNullParam(yController, "yController", "SwerveControllerCommand"),
            requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand")),
        desiredRotation,
        subsystem);
  }

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path.
   * This is left to the user since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param subsystem The subsystems to require.
   */
  public SwerveControllerCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      SwerveSubsystem subsystem) {
    this(
        trajectory,
        pose,
        xController,
        yController,
        thetaController,
        () ->
            trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        subsystem);
    this.m_xController = xController;
    this.m_yController = yController;
    this.m_thetaController = thetaController;
    this.m_trajectory = trajectory;

  }

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param controller The HolonomicDriveController for the drivetrain.
   * @param subsystem The subsystems to require.
   */
  public SwerveControllerCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      HolonomicDriveController controller,
      SwerveSubsystem subsystem) {
    this(
        trajectory,
        pose,
        controller,
        () ->
            trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        subsystem);
  }

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param controller The HolonomicDriveController for the drivetrain.
   * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
   *     time step.
   * @param subsystem The subsystems to require.
   */
  public SwerveControllerCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      HolonomicDriveController controller,
      Supplier<Rotation2d> desiredRotation,
      SwerveSubsystem subsystem) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
    m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
    m_controller = requireNonNullParam(controller, "controller", "SwerveControllerCommand");

    m_desiredRotation =
        requireNonNullParam(desiredRotation, "desiredRotation", "SwerveControllerCommand");

    this.m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);

    var targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
    
    m_subsystem.drive(targetChassisSpeeds, new Translation2d());
  }

    @Override
    public void end(boolean interrupted) {
      m_timer.stop();
      m_subsystem.stop();
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }


    public SwerveControllerCommand copyOf() {
      return new SwerveControllerCommand(m_trajectory,
      m_pose,
      m_xController,
      m_yController,
      m_thetaController,
      m_subsystem);
    }
}
