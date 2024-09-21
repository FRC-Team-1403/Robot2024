package team1403.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CougarUtil {
    
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static boolean shouldMirrorPath() {
        return getAlliance() == Alliance.Red;
    }
}
