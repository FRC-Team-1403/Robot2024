
package team1403.robot.swerve;

import javax.swing.tree.ExpandVetoException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private PhotonCamera limeLight;
  private PhotonPipelineResult result;
  private AprilTagFieldLayout fieldLayout;


  private double cameraHeightMeters = 0.559;
  private double targetHeightMeters = 1.3208;
  private double cameraPitchDegrees = -35;

  public Limelight() {
    // Photonvision
    PortForwarder.add(5800, "photonvision.local", 5800);
    limeLight = new PhotonCamera("1403Camera");

    // 0: April Tags
    // 1: Reflective Tape
    limeLight.setPipelineIndex(0);
    
    result = new PhotonPipelineResult();

    try
    {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    }
    catch(Exception e)
    {
      System.err.println(e.getMessage());
    }
    //Cone detection
  }

  public double getZDistance() {
    result = limeLight.getLatestResult();
    return result.hasTargets() ? result.getBestTarget().getBestCameraToTarget().getZ() : 0;  
  }

  public double getXDistance() {
    result = limeLight.getLatestResult();
    return result.hasTargets() ? result.getBestTarget().getBestCameraToTarget().getX() : 0;
  }

  public double getDistanceFromTarget() {
    result = limeLight.getLatestResult();
    if (result.hasTargets()) {
      double distanceToTarget =  PhotonUtils.calculateDistanceToTargetMeters(
        cameraHeightMeters,
        targetHeightMeters,
        Units.degreesToRadians(cameraPitchDegrees),
        result.getBestTarget().getBestCameraToTarget().getRotation().getAngle());
      return distanceToTarget;
    }
    return 0;
  }

  public double getYDistance() {
    result = limeLight.getLatestResult();
    return result.hasTargets() ? result.getBestTarget().getBestCameraToTarget().getY() : 0;
  }

  public double getZAngle() {
    return Math.acos(getZDistance() / getDistanceFromTarget());
  }
  
  public double getXAngle(){
    return result.getBestTarget().getSkew();
  }

  public double getYAngle(){
    return result.getBestTarget().getYaw();
  }

  public double getAmbiguity(){
    return result.getBestTarget().getPoseAmbiguity();
  }

  public boolean hasTarget(){
    return result.hasTargets();
  }

  public Pose3d getDistance(){
    if(hasTarget())
        return PhotonUtils.estimateFieldToRobotAprilTag(result.getBestTarget().getBestCameraToTarget(), fieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get(), new Transform3d(0.0,0.0,0.0, new Rotation3d(0.0,0.0,0.0)));
    else
        return null;
  }

  public Pose2d getDistance2D() {
    Pose3d pose = getDistance();
    Rotation3d rot = pose.getRotation();
    return new Pose2d(pose.getX(), pose.getY(), new Rotation2d(rot.getZ()));
  }

  public Matrix<N3,N1> getPosStdv(){
    return new Matrix<N3,N1>(VecBuilder.fill(getXDistance()*getAmbiguity(),getYDistance()*getAmbiguity(),result.getBestTarget().getBestCameraToTarget().getRotation().getAngle()*getAmbiguity()));
    
  }

  @Override
  public void periodic() {
    result = limeLight.getLatestResult();
    if(hasTarget())
    {
      SmartDashboard.putString("pos", getDistance().toString());
    }
  }
}
