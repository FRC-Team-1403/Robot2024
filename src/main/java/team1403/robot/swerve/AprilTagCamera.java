
package team1403.robot.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class AprilTagCamera extends SubsystemBase {
  private final PhotonCamera m_camera;
  private PhotonPipelineResult m_result;
  private PhotonPoseEstimator m_poseEstimator;
  private Supplier<Transform3d> m_cameraTransform;
  private EstimatedRobotPose m_estPos;
  private Supplier<Pose2d> m_referencePose;
  private Pose2d m_Pose2d = null;
  private static final Matrix<N3, N1> kDefaultStdv = VecBuilder.fill(0.9, 0.9, 999999);
  private static final boolean kExtraVisionDebugInfo = true;

  public AprilTagCamera(String cameraName, Supplier<Transform3d> cameraTransform, Supplier<Pose2d> referenceSupplier) {
    // Photonvision
    // PortForwarder.add(5800, 
    // "photonvision.local", 5800);
    m_camera = new PhotonCamera(cameraName);

    // 0: April Tags


    // 1: Reflective Tape
    m_camera.setPipelineIndex(0);
    
    m_result = new PhotonPipelineResult();

    m_poseEstimator = new PhotonPoseEstimator(Constants.Vision.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, cameraTransform.get());
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

    m_estPos = null;
    m_referencePose = referenceSupplier;
    m_cameraTransform = cameraTransform;
    //Cone detection
  }

  @Override
  public String getName() {
    return m_camera.getName();
  }

  public boolean hasTarget() {
    return m_result.hasTargets();
  }

  public boolean hasPose() {
    return m_estPos != null;
  }

  public Pose3d getPose() {
    if(hasPose())
    {
      return m_estPos.estimatedPose;
    }
    return null;
  }

  private void updatePose2d() {
    Pose3d pose = getPose();
    if(pose == null)
      m_Pose2d = null;
    else
      m_Pose2d = pose.toPose2d();
  }

  public Pose2d getPose2D() {
    return m_Pose2d;
  }

  //gets the timestamp of the latest pose
  public double getTimestamp() {
    if(hasPose())
    {
      return m_estPos.timestampSeconds;
    }
    return -1;
  }

  public List<PhotonTrackedTarget> getTargets() {
    if(hasPose())
    {
      return m_estPos.targetsUsed;
    }
    return new ArrayList<>();
  }

  private double getTagAreas() {
    double ret = 0;
    if(!hasPose()) return 0;
    for(PhotonTrackedTarget t : getTargets()) {
      ret += t.getArea();
    }
    return ret;
  }

  //default values, TODO: update later to compute the actual estimated standard deviation
  public Matrix<N3, N1> getEstStdv() {
    return kDefaultStdv;
  }

  //TODO: return false for bad estimates
  public boolean checkVisionResult() {

    if(getTagAreas() < 0.2) return false;

    return true;
  }

  private ArrayList<Pose3d> m_visionTargets = new ArrayList<>();
  private static final Transform3d kZeroTransform = new Transform3d();

  @Override
  public void periodic() {
    if(m_camera.isConnected()) {
      m_result = m_camera.getLatestResult();

      m_poseEstimator.setReferencePose(m_referencePose.get());
      m_poseEstimator.setRobotToCameraTransform(m_cameraTransform.get());
      m_estPos = m_poseEstimator.update(m_result).orElse(null);

      updatePose2d();

      Logger.recordOutput(m_camera.getName() + "/Target Visible", hasTarget());

      if(kExtraVisionDebugInfo) {
        Pose3d robot_pose3d = new Pose3d(m_referencePose.get());
        Pose3d robot_pose_transformed = robot_pose3d.transformBy(m_cameraTransform.get());

        Logger.recordOutput(m_camera.getName() + "/Camera Transform", robot_pose_transformed);

        m_visionTargets.clear();

        for(PhotonTrackedTarget t : getTargets()) {
          var trf = t.getBestCameraToTarget();
          if(trf.equals(kZeroTransform)) continue;
          m_visionTargets.add(robot_pose_transformed.transformBy(trf));
        }

        Logger.recordOutput(m_camera.getName() + "/Vision Targets", m_visionTargets.toArray(new Pose3d[m_visionTargets.size()]));
      }
      
      if(hasPose()) {
        Logger.recordOutput(m_camera.getName() + "/Pose3d", getPose());
        Logger.recordOutput(m_camera.getName() + "/Pose2d", getPose2D());
        Logger.recordOutput(m_camera.getName() + "/Combined Area", getTagAreas());
      }
    }

    // if(hasTarget())
    // {
    //   SmartDashboard.putString("pos", getDistance().toString());
    //   Logger.recordOutput("Position", getDistance().toString());
    //   Logger.recordOutput("X Distance", getXDistance());
    //   Logger.recordOutput("Y Distance", getYDistance());
    //   Logger.recordOutput("Z Distance", getZDistance());
    }
  }
