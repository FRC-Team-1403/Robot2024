package team1403.robot.swerve;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import team1403.robot.Constants;
import team1403.robot.Robot;

public class VisionSimUtil {
    private static VisionSystemSim visionSim;

    public static void initVisionSim() {
        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(Constants.Vision.kFieldLayout);
        }
    }

    public static void addCamera(PhotonCameraSim simCam, Transform3d robotToCamera) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.addCamera(simCam, robotToCamera);
        }
    }

    public static void adjustCamera(PhotonCameraSim simCam, Transform3d robotToCamera) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.adjustCamera(simCam, robotToCamera);
        }
    }

    public static void update(Pose2d pose) {
        if(Robot.isSimulation() && visionSim != null) {
            visionSim.update(pose);
        }
    }
}
