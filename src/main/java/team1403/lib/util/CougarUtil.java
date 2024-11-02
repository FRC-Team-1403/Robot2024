package team1403.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CougarUtil {
    
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static boolean shouldMirrorPath() {
        return getAlliance() == Alliance.Red;
    }

    public static Pose2d createPose2d(Pose2d pose, Rotation2d rot) {
        return new Pose2d(pose.getTranslation(), rot);
    }

    public static Pose2d getInitialRobotPose() {
        if(getAlliance() == Alliance.Red)
            return new Pose2d(new Translation2d(), new Rotation2d(Math.PI));
        
        return new Pose2d();
    }
}
