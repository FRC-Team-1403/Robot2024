package team1403.lib.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team1403.robot.Constants;
import team1403.robot.swerve.SwerveSubsystem;

public class AutoUtil {
    
  public static Command loadChoreoAuto(String name, SwerveSubsystem swerve) {
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
    Command cmd = AutoBuilder.followPath(path);
    return Commands.sequence(Commands.runOnce(() -> {
        if(Constants.kAllianceSupplier.get() == Alliance.Blue) {
            swerve.resetOdometry(path.getStartingDifferentialPose());
        }
        else {
            swerve.resetOdometry(path.flipPath().getStartingDifferentialPose());
        }
    }, swerve), cmd);
  }

  public static Command pathFindToPose(Pose2d target) {
    return AutoBuilder.pathfindToPose(target, Constants.Swerve.kPathConstraints);
  }

    
}
