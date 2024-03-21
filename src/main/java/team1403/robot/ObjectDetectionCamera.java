package team1403.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import team1403.PathUtils;

/** Wrapper for PhotonCamera class */
public class ObjectDetectionCamera extends PhotonCamera {

    // Camera angles for calculating target distance.
    // Used for https://javadocs.photonvision.org/org/photonvision/PhotonUtils.html#calculateDistanceToTargetMeters(double,double,double,double)
    private static final String DEFAULT_CAM_NAME = "ObjectDetectionCam";
    private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(18.0); // height on robot (meters) TODO: CHECK THIS
    private final double CAMERA_PITCH_RADIANS = Rotation2d.fromDegrees(-25.0).getRadians(); // tilt of our camera (radians)
    private final double TARGET_HEIGHT_METERS = 0.0; // may need to change 

    public ObjectDetectionCamera() {
        super(DEFAULT_CAM_NAME);
    }

    public double getYaw() {
        /* The yaw of the target in degrees (positive right). */
        var result = getLatestResult();
        return result.hasTargets() ? 
            result.getBestTarget().getYaw() :
            0.0;
    }

    public double getPitch() {
        /* The pitch of the target in degrees (positive up). */
        var result = getLatestResult();
        return result.hasTargets() ? 
            result.getBestTarget().getPitch() :
            0.0;
    }

    public double getSkew() {
        /* The skew of the target in degrees (counter-clockwise positive). */
        var result = getLatestResult();
        return result.hasTargets() ? 
            result.getBestTarget().getSkew() :
            0.0;
    }

    public int getTargetCount() {
        var result = getLatestResult();
        return result.hasTargets() ? 
            result.getTargets().size() :
            0;
    }

    public double getDistanceToTarget() {
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
                Units.degreesToRadians(result.getBestTarget().getPitch())
            );
            return range;
        }
        return 0.0;
    }
    // Not sure if this will work
    public Pose2d transformToNote() {
        Transform3d pose = getLatestResult().getBestTarget().getBestCameraToTarget();
        return new Pose2d(pose.getX(), pose.getY(),pose.getRotation().toRotation2d());
    }
     public Transform3d transformToNote(int x) {
        Transform3d pose = getLatestResult().getBestTarget().getBestCameraToTarget();
        return pose;
    }
    //this will work
    public Pose2d getNotePose2d(Pose2d currentPose) {
        Pose2d notePose = new Pose2d(TargetUtils.getNoteTranslation(this, currentPose,this.getDistanceToTarget() + Units.inchesToMeters(4.0)), TargetUtils.getTargetHeadingToClosestNote(this, currentPose));
        return  notePose;
    }
    public Trajectory getPathToNote(Pose2d currentPose) {
        PathPlannerPath pathToNote = PathUtils.generatePath(currentPose, this.getNotePose2d(currentPose));
        return PathUtils.TrajectoryFromPath(pathToNote);
    }
}