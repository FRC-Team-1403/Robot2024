package team1403.lib.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team1403.robot.Constants.Swerve;
import team1403.robot.swerve.SwerveSubsystem;

public class AutoUtil {

  private static final PathConstraints kPathConstraints = new PathConstraints(Swerve.kMaxSpeed, 4, Swerve.kMaxAngularSpeed, 10);
    
  public static Command loadChoreoAuto(String name, SwerveSubsystem swerve) {
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
    Command cmd = AutoBuilder.followPath(path);
    return Commands.sequence(Commands.runOnce(() -> {
        if(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue) {
            swerve.resetOdometry(path.getStartingDifferentialPose());
        }
        else {
            swerve.resetOdometry(path.flipPath().getStartingDifferentialPose());
        }
    }, swerve), cmd);
  }

  public static Command pathFindToPose(Pose2d target) {
    return AutoBuilder.pathfindToPose(target, kPathConstraints);
  }

    
}
