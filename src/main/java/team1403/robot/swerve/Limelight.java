
package team1403.robot.swerve;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private PhotonCamera limeLight;
  private PhotonPipelineResult result;


  private double cameraHeightMeters = 0.559;
  private double targetHeightMeters = 1.3208;
  private double cameraPitchDegrees = -35;

  public Limelight() {
    // Photonvision
    PortForwarder.add(5800, "photonvision.local", 5800);
    limeLight = new PhotonCamera("Team1403Camera");

    // 0: April Tags
    // 1: Reflective Tape
    limeLight.setPipelineIndex(0);
    
    result = new PhotonPipelineResult();

    //Cone detection
  }

  public double getZDistance() {
    return Math.pow(getDistanceFromTarget(), 2) - Math.pow(getXDistance(), 2);
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

  public Pose2d getDistance(){
    return new Pose2d(new Translation2d(getXDistance(),getYDistance()), new Rotation2d(getXAngle(),getYAngle()));
  }

  public Matrix<N3,N1> getPosStdv(){
    return new Matrix<N3,N1>(VecBuilder.fill(getXDistance()*getAmbiguity(),getYDistance()*getAmbiguity(),result.getBestTarget().getBestCameraToTarget().getRotation().getAngle()*getAmbiguity()));
    
  }

  @Override
  public void periodic() {
  }
}
