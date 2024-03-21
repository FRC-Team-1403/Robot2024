package team1403.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TargetUtils {

    // public static double getDistanceToFieldPos(Pose2d robotPose, int apriltag) {
    //     double distance = 0.0;
    //     Optional<Pose3d> tagPose = KnownLocations.getFieldLayout().getTagPose(apriltag);

    //     if (tagPose.isPresent()) {
    //         distance = robotPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());
    //     }

    //     return distance;
    // }

    // public static double getDistanceToSpeaker(Pose2d robotPose) {
    //     Alliance alliance = KnownLocations.getKnownLocations().alliance;
    //     if (alliance == Alliance.Blue) {
    //         return getDistanceToFieldPos(robotPose, APRILTAGS.MIDDLE_BLUE_SPEAKER);
    //     } else {
    //         return getDistanceToFieldPos(robotPose, APRILTAGS.MIDDLE_RED_SPEAKER);
    //     }
    // }

    // // Following is based off https://www.chiefdelphi.com/t/is-there-a-builtin-function-to-find-the-angle-needed-to-get-one-pose2d-to-face-another-pose2d/455972
    // public static double getTargetHeadingToAprilTag(Pose2d robotPose, int tagId) {
    //     double heading = 0.0;
    //     Optional<Pose3d> tagPose = KnownLocations.getFieldLayout().getTagPose(tagId);
    //     if (tagPose.isPresent()) {
    //         Translation2d tagPoint = tagPose.get().getTranslation().toTranslation2d();
    //         Rotation2d targetRotation = tagPoint.minus(robotPose.getTranslation()).getAngle();
    //         heading = targetRotation.rotateBy(Rotation2d.fromDegrees(180.0)).getDegrees();
    //     }
    //     return heading;
    // }

    // public static double getTargetHeadingToFieldPosition(Pose2d robotPose, FieldPosition fieldPos) {
    //     double heading = 0.0;
    //     Alliance alliance = KnownLocations.getKnownLocations().alliance;
        
    //     if (fieldPos == FieldPosition.SPEAKER) {
    //         int apriltag = (alliance == Alliance.Blue) ?
    //             APRILTAGS.MIDDLE_BLUE_SPEAKER :
    //             APRILTAGS.MIDDLE_RED_SPEAKER;
    //         heading = getTargetHeadingToAprilTag(robotPose, apriltag);
    //     } else if (fieldPos == FieldPosition.AMP) {
    //         int apriltag = (alliance == Alliance.Blue) ?
    //             APRILTAGS.BLUE_AMP :
    //             APRILTAGS.RED_AMP;
    //         heading = getTargetHeadingToAprilTag(robotPose, apriltag);       
    //     }
    //     //should never get here
    //     return heading;
    // }

    public static Rotation2d getTargetHeadingToClosestNote(ObjectDetectionCamera objCam, Pose2d robotPose) {
        Rotation2d targetRotation = Rotation2d.fromDegrees(-objCam.getYaw() + 2.0);
        return objCam.getYaw() != 0.0 ?
            robotPose.rotateBy(targetRotation).getRotation() :
            robotPose.getRotation();
    }

   public static Translation2d getNoteTranslation(ObjectDetectionCamera objCam, Pose2d robotPose, double distance) {
        return new Translation2d(distance + Units.inchesToMeters(5.0), getTargetHeadingToClosestNote(objCam, robotPose)).plus(robotPose.getTranslation());
   }

//    public static boolean isInWing(Pose2d pose) {
//         KnownLocations knownLocations = KnownLocations.getKnownLocations();
//         Alliance alliance = knownLocations.alliance;

//         return alliance == Alliance.Blue ?
//             pose.getX() > knownLocations.WING_NOTE_TOP.getX() + 50.0:
//             pose.getX() < knownLocations.WING_NOTE_TOP.getX() - 50.0;
//    }

//    // TODO: Create KnownLocations for the X values in this method
//    public static boolean isInShootingZone(Pose2d pose) {
//        boolean inShootingZone = false;
//        Alliance alliance = KnownLocations.getKnownLocations().alliance;

//        if (!isInStageArea(pose)) {
//            if (alliance == Alliance.Blue) {
//                inShootingZone = Units.metersToInches(pose.getX()) < 230.0;
//            } else if (alliance == Alliance.Red) {
//                inShootingZone = Units.metersToInches(pose.getX()) > 420.0;
//            }
//        }
//        return inShootingZone;
//    }

//    // TODO: Create KnownLocations for the X/Y values in this method
//    public static boolean isInStageArea(Pose2d pose) {
//        boolean inStageArea = false;
//        Alliance alliance = KnownLocations.getKnownLocations().alliance;

//        double x = Units.metersToInches(pose.getX());
//        double y = Units.metersToInches(pose.getY());
//        if (alliance == Alliance.Blue) {
//            inStageArea = (y > ((-0.57735 * x) + 227.16483)) &&
//                    (y < ((0.57735 * x) + 96.85518)) &&
//                    (x < 230.0);
//        } else if (alliance == Alliance.Red) {
//            inStageArea = (y < ((-0.57735 * x) + 473.10859)) &&
//                    (y > ((0.57735 * x) - 149.10859)) &&
//                    (x > 420.0);
//        }

//        return inStageArea;
//    }

//    public static Optional<Rotation2d> getRotationTargetOverride(Drivetrain drivetrain, Intake intake, Arm arm){
//     // Some condition that should decide if we want to override rotation
//     var result = drivetrain.getObjCam().getLatestResult();
//     if(result.hasTargets() && !intake.hasNote() && arm.isAtPosition(ArmPosition.HOME)) {
//         // Return an optional containing the rotation override (this should be a field relative rotation)
//         return Optional.of(getTargetHeadingToClosestNote(drivetrain.getObjCam(), drivetrain.getPose()));
//     } else if (intake.hasNote()) {
//         // return an empty optional when we don't want to override the path's rotation
//         return Optional.of(Rotation2d.fromDegrees(getTargetHeadingToFieldPosition(drivetrain.getPose(), FieldPosition.SPEAKER)));
//     } else {
//         return Optional.empty();
//     }
// }

}