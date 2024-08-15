
package team1403.robot.swerve;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class AprilTagCamera extends SubsystemBase {
  private final PhotonCamera m_camera;
  private PhotonPipelineResult m_result;
  private PhotonPoseEstimator m_poseEstimator;
  private Optional<EstimatedRobotPose> m_estPos;
  private Supplier<Pose2d> m_referencePose;

  public AprilTagCamera(String cameraName, Transform3d cameraTransform, Supplier<Pose2d> referenceSupplier) {
    // Photonvision
    // PortForwarder.add(5800, 
    // "photonvision.local", 5800);
    m_camera = new PhotonCamera(cameraName);

    // 0: April Tags


    // 1: Reflective Tape
    m_camera.setPipelineIndex(0);
    
    m_result = new PhotonPipelineResult();

    m_poseEstimator = new PhotonPoseEstimator(Constants.Vision.kFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, m_camera, cameraTransform);
    m_estPos = Optional.empty();
    m_referencePose = referenceSupplier;
    //Cone detection
  }

  public boolean hasTarget() {
    return m_result.hasTargets();
  }

  //must be called after checking if there is a target!
  public double getTagAmbiguity() {
    return m_result.getBestTarget().getPoseAmbiguity();
  }

  public Pose3d getPose() {
    if(m_estPos.isPresent())
    {
      return m_estPos.get().estimatedPose;
    }
    return null;
  }

  public Pose2d getPose2D() {
    Pose3d pose = getPose();
    if(pose == null)
      return null;
    return pose.toPose2d();
  }

  //gets the timestamp of the latest pose
  public double getTimestamp() {
    if(m_estPos.isPresent())
    {
      return m_estPos.get().timestampSeconds;
    }
    return -1;
  }

  @Override
  public void periodic() {
    m_result = m_camera.getLatestResult();

    m_poseEstimator.setReferencePose(m_referencePose.get());
    m_estPos = m_poseEstimator.update(m_result);

    Logger.recordOutput("Target Visible", hasTarget());

    // if(hasTarget())
    // {
    //   SmartDashboard.putString("pos", getDistance().toString());
    //   Logger.recordOutput("Position", getDistance().toString());
    //   Logger.recordOutput("X Distance", getXDistance());
    //   Logger.recordOutput("Y Distance", getYDistance());
    //   Logger.recordOutput("Z Distance", getZDistance());
    }
  }
