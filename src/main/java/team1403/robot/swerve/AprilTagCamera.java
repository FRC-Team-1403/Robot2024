
package team1403.robot.swerve;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
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
  private final Transform3d m_cameraTransform;

  public AprilTagCamera(String cameraName, Transform3d cameraTransform) {
    // Photonvision
    // PortForwarder.add(5800, 
    // "photonvision.local", 5800);
    m_camera = new PhotonCamera(cameraName);

    // 0: April Tags


    // 1: Reflective Tape
    m_camera.setPipelineIndex(0);
    
    m_result = new PhotonPipelineResult();

    m_cameraTransform = cameraTransform;
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
    if(hasTarget())
    {
      PhotonTrackedTarget target = m_result.getBestTarget();
      Optional<Pose3d> pose = Constants.Vision.kFieldLayout.getTagPose(target.getFiducialId());
      if(pose.isEmpty())
      {
        System.err.println("RIP code");
        return null;
      }
      return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), 
      pose.get(), m_cameraTransform);
    }
    else
        return null;
  }

  public Pose2d getPose2D() {
    Pose3d pose = getPose();
    if(pose == null)
      return null;
    Rotation3d rot = pose.getRotation();
    return new Pose2d(pose.getX(), pose.getY(), new Rotation2d(rot.getZ()));
  }

  @Override
  public void periodic() {
    m_result = m_camera.getLatestResult();
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
