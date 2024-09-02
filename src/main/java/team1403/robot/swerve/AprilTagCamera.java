
package team1403.robot.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;
import team1403.robot.Constants.Vision;

public class AprilTagCamera extends SubsystemBase implements IAprilTagCamera {
  private final PhotonCamera m_camera;
  private final PhotonCameraSim m_cameraSim;
  private PhotonPipelineResult m_result;
  private PhotonPoseEstimator m_poseEstimator;
  private Supplier<Transform3d> m_cameraTransform;
  private Optional<EstimatedRobotPose> m_estPos;
  private Supplier<Pose2d> m_referencePose;
  private static final Matrix<N3, N1> kDefaultStdv = VecBuilder.fill(0.9, 0.9, 0.9);
  private static final boolean kExtraVisionDebugInfo = true;

  public AprilTagCamera(String cameraName, Supplier<Transform3d> cameraTransform, Supplier<Pose2d> referenceSupplier) {
    // Photonvision
    // PortForwarder.add(5800, 
    // "photonvision.local", 5800);
    m_camera = new PhotonCamera(cameraName);

    SimCameraProperties cameraProp = new SimCameraProperties();

    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(63));
    // Approximate detection noise with average and standard deviation error in pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop rate).
    cameraProp.setFPS(30);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    m_cameraSim = new PhotonCameraSim(m_camera, cameraProp);

    Constants.Vision.kVisionSystem.addCamera(m_cameraSim, cameraTransform.get());

    // 0: April Tags


    // 1: Reflective Tape
    m_camera.setPipelineIndex(0);
    
    m_result = new PhotonPipelineResult();

    m_poseEstimator = new PhotonPoseEstimator(Constants.Vision.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, cameraTransform.get());
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

    m_estPos = Optional.empty();
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
    return m_estPos.isPresent();
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

  public List<PhotonTrackedTarget> getTargets() {
    if(m_estPos.isPresent())
    {
      return m_estPos.get().targetsUsed;
    }
    return new ArrayList<>();
  }

  public double getTagArea() {
    if(hasPose()) {
      double sum = 0;
      for(PhotonTrackedTarget t : getTargets()) {
        sum += t.getArea();
      }
      return sum;
    }
    return 0;
  }

  public boolean checkVisionResult() {

    if(!hasPose()) return false;

    Pose3d pose = getPose();

    if(pose.getZ() >= 1.5) return false;

    if(getTagArea() <= 0.2) return false;

    return true;
  }

  //default values, TODO: update later to compute the actual estimated standard deviation
  public Matrix<N3, N1> getEstStdv() {
    return kDefaultStdv;
  }

  private ArrayList<Pose3d> m_visionTargets = new ArrayList<>();
  private ArrayList<Pose2d> m_visionTargets2d = new ArrayList<>();
  private static final Transform3d kZeroTransform = new Transform3d();

  @Override
  public void periodic() {
    m_result = m_camera.getLatestResult();

    m_poseEstimator.setReferencePose(m_referencePose.get());
    m_poseEstimator.setRobotToCameraTransform(m_cameraTransform.get());
    Vision.kVisionSystem.adjustCamera(m_cameraSim, m_cameraTransform.get());
    m_estPos = m_poseEstimator.update(m_result);

    Logger.recordOutput(m_camera.getName() + "/Target Visible", hasTarget());

    if(kExtraVisionDebugInfo) {
      Pose3d robot_pose3d = new Pose3d(m_referencePose.get());
      Pose3d robot_pose_transformed = robot_pose3d.transformBy(m_cameraTransform.get());

      Logger.recordOutput(m_camera.getName() + "/Camera Transform", robot_pose_transformed);

      m_visionTargets.clear();
      m_visionTargets2d.clear();

      List<PhotonTrackedTarget> targets = getTargets();
      for(PhotonTrackedTarget t : targets) {
        var trf = t.getBestCameraToTarget();
        if(trf.equals(kZeroTransform)) continue;
        Pose3d vision_target = robot_pose_transformed.transformBy(trf);
        m_visionTargets.add(vision_target);
        m_visionTargets2d.add(vision_target.toPose2d());
      }

      Logger.recordOutput(m_camera.getName() + "/Vision Targets", m_visionTargets.toArray(new Pose3d[m_visionTargets.size()]));
      Logger.recordOutput(m_camera.getName() + "/Vision Targets2d", m_visionTargets2d.toArray(new Pose2d[m_visionTargets.size()]));
    }
    
    if(hasPose()) {
      Logger.recordOutput(m_camera.getName() + "/Pose3d", getPose());
      Logger.recordOutput(m_camera.getName() + "/Pose2d", getPose2D());
      Logger.recordOutput(m_camera.getName() + "/Area", getTagArea());
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
