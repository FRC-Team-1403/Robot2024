
package team1403.robot.swerve;

import java.util.Optional;

import javax.swing.tree.ExpandVetoException;

import org.littletonrobotics.junction.Logger;
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
import team1403.robot.Constants;

public class Limelight extends SubsystemBase {
  private PhotonCamera limeLight;
  private PhotonPipelineResult result;
  private AprilTagFieldLayout fieldLayout;


  private double cameraHeightMeters = 0.559;
  private double targetHeightMeters = 1.3208;
  private double cameraPitchDegrees = -35;
  private int targetCount = 0;

  public Limelight() {
    // Photonvision
    PortForwarder.add(5800, 
    "photonvision.local", 5800);
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

  // public double getTagXPos(){
  //   return Constants.Vision.fieldLayout.;
  // }


  public double getZDistance() {
    return result.hasTargets() ? fieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getZ() : 0;  
  }

  public double getXDistance() {
    return result.hasTargets() ? fieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getX() : 0;  
  }

  public double getDistanceFromTarget() {
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
    return result.hasTargets() ? result.getBestTarget().getBestCameraToTarget().getY() : 0;
  }

  public double getZAngle() {
    //  return Units.degreesToRadians(result.hasTargets() ? fieldLayout.getTagPose((result.getBestTarget().getFiducialId())).get().getRotation().getZ()  : 0);
    double angle = result.hasTargets() ? result.getBestTarget().getBestCameraToTarget().getRotation().getAngle() : 0;
    return result.hasTargets() ? Units.radiansToDegrees(angle) : 0;
   
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
    {
      Optional<Pose3d> pose = fieldLayout.getTagPose(result.getBestTarget().getFiducialId());
      if(pose.isEmpty())
      {
        System.err.println("RIP code");
        return null;
      }
      return PhotonUtils.estimateFieldToRobotAprilTag(result.getBestTarget().getBestCameraToTarget(), 
      pose.get(), Constants.Swerve.kCameraOffset);
    }
    else
        return null;
  }

  public Pose2d getDistance2D() {
    Pose3d pose = getDistance();
    Rotation3d rot = pose.getRotation();
    return new Pose2d(pose.getX(), pose.getY(), new Rotation2d(rot.getZ()));
  }

  @Override
  public void periodic() {
    // result = limeLight.getLatestResult();
    
    // Logger.recordOutput("Target Visible?", hasTarget());
    
    // if(hasTarget())
    // {
    //   SmartDashboard.putString("pos", getDistance().toString());
    //   Logger.recordOutput("Position", getDistance().toString());
    //   Logger.recordOutput("X Distance", getXDistance());
    //   Logger.recordOutput("Y Distance", getYDistance());
    //   Logger.recordOutput("Z Distance", getZDistance());
    }
  }
