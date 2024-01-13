
package team1403.robot.swerve;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private PhotonCamera limeLight;
  private PhotonPipelineResult result;

  private int pipelineIndex;

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
    return result.hasTargets() ? result.getBestTarget().getYaw() : 0;
  }

  public double getDistanceFromTarget() {
    result = limeLight.getLatestResult();
    if (result.hasTargets()) {
      double distanceToTarget =  PhotonUtils.calculateDistanceToTargetMeters(
        cameraHeightMeters,
        targetHeightMeters,
        Units.degreesToRadians(cameraPitchDegrees),
        Units.degreesToRadians(result.getBestTarget().getPitch()));
      return distanceToTarget;
    }
    return 0;
  }

  public double getZAngle() {
    return Math.acos(getZDistance() / getDistanceFromTarget());
  }

  @Override
  public void periodic() {
  }
}
