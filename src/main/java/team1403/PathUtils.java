package team1403;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import team1403.robot.Constants;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class PathUtils {

    /**
     * 
     */
    public static final PathConstraints pathConstraints = new PathConstraints(
        Constants.Swerve.kMaxSpeed * 0.2,
        .5,
        .5,
        Constants.Swerve.kMaxAngularSpeed * .1
    );

    /**
     * Generate a PathPlannerPath from a supplied list of poses.
     * <P>
     * Convenience method to generate a PathPlannerPath from a list of poses. The
     * rotation of the last pose us used for the end state rotation of the path.
     * The required path constraints are taken from the swerve drive constants.
     * @param poses The poses of the waypoints for the path.
     * @return The constructed path.
     */
    public static PathPlannerPath generatePath(Pose2d... poses) {
        return PathUtils.generatePath(poses[poses.length-1].getRotation(), poses);
    }

    /**
     * Generate a PathPlannerPath from a supplied list of poses.
     * <P>
     * Convenience method to generate a PathPlannerPath from a provided end state
     * rotation and a list of poses. The required path constraints are taken from
     * the swerve drive constants.
     * @param endStateRotation The desired end state rotation (not heading).
     * @param poses The poses of the waypoints for the path.
     * @return The constructed path.
     */
    public static PathPlannerPath generatePath(Rotation2d endStateRotation, Pose2d... poses) {   
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);
        return new PathPlannerPath(
            bezierPoints,
            PathUtils.pathConstraints,
            new GoalEndState(0.0, endStateRotation, true));
    }

    /**
     * Generate a WPILib trajectory from a PathPlannerPath for display on a Field2d widget.
     * <P>
     * NOTE: This should only be used for displaying a PathPlannerPath as a Trajectory on
     * a Field2d widget. The generated trajectory is not meant to run on the robot.
     * @param path The PathPlannerPath to generate the Trajectory from.
     * @return The generated Trajectory.
     */
    public static Trajectory TrajectoryFromPath(PathPlannerPath path) {
        PathPlannerTrajectory ppTrajectory = path.getTrajectory(new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation());
        List<Trajectory.State> states = new ArrayList<Trajectory.State>();
        for(PathPlannerTrajectory.State pState : ppTrajectory.getStates()) {
            states.add(new Trajectory.State(
                pState.timeSeconds,
                pState.velocityMps,
                pState.accelerationMpsSq,
                new Pose2d(pState.positionMeters, pState.heading),
                pState.curvatureRadPerMeter                
            ));
        }
        return new Trajectory(states);
    }


}