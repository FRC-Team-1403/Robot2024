package team1403.robot.autos;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team1403.lib.auto.TreeAuto;
import team1403.lib.auto.TreeCommandNode;
import team1403.lib.auto.TreeCommandProxy;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.swerve.SwerveSubsystem;

public class AutoHelper {
    
    //NOTE:
    //left is success
    //right is fail

    private static TreeCommandNode loadPathResetPose(String name, BooleanSupplier success, SwerveSubsystem swerve) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(name);
        Command cmd = AutoBuilder.followPath(path);
        Command resetcmd = swerve.runOnce(
            () -> {
                PathPlannerPath flipped = path;
                if (CougarUtil.shouldMirrorPath()) flipped = path.flipPath();
                swerve.resetOdometry(flipped.getStartingDifferentialPose());
            }
        );
        return new TreeCommandProxy(Commands.sequence(resetcmd, cmd), success);
    }

    //path find to fix "bad" transitions between paths
    private static TreeCommandNode loadPath(String name, BooleanSupplier success) {
        return new TreeCommandProxy(
            AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(name), 
        Constants.Swerve.kPathConstraints), success);
    }

    private static TreeCommandNode loadPath(String name) {
        return loadPath(name, () -> true);
    }

    public static Command getOnePiece() {
        return Blackbox.shoot();
    }

    //potentially rewrite to use Commands.either :)
    public static Command getFivePieceAuto(SwerveSubsystem swerve) {
        //shoot is always success
        TreeCommandNode shoot = new TreeCommandProxy(Blackbox.shoot());

        //end of auto (TODO: make the 4th piece go further out)
        TreeCommandNode fourthPieceShoot = loadPath("fourthPieceShoot").setNext(shoot.clone());
        TreeCommandNode fourthPieceFromThird = loadPath("fourthPieceFromThird", () -> Blackbox.isLoaded()).setNext(fourthPieceShoot);
        TreeCommandNode fourthPiece = loadPath("fourthPiece", () -> Blackbox.isLoaded()).setNext(fourthPieceShoot);

        TreeCommandNode thirdPieceShoot = loadPath("thirdPieceShoot").setNext(shoot.clone().setNext(fourthPiece));
        TreeCommandNode thirdPiece = loadPath("thirdPiece", () -> Blackbox.isLoaded()).setNext(thirdPieceShoot, fourthPieceFromThird);
        TreeCommandNode thirdPieceFromSecond = loadPath("thirdPieceFromSecond", () -> Blackbox.isLoaded()).setNext(thirdPieceShoot, fourthPieceFromThird);
        TreeCommandNode secondPieceShoot = loadPath("secondPieceShoot").setNext(shoot.clone().setNext(thirdPiece));
        TreeCommandNode secondPiece = loadPath("secondPiece", () -> Blackbox.isLoaded()).setNext(secondPieceShoot, thirdPieceFromSecond);
        TreeCommandNode secondPieceFromFirst = loadPath("secondPieceFromFirst", () -> Blackbox.isLoaded()).setNext(secondPieceShoot, thirdPieceFromSecond);
        TreeCommandNode firePieceShoot = loadPath("firstPieceShoot").setNext(shoot.clone().setNext(secondPiece));
        TreeCommandNode firstPiece = loadPathResetPose("firstPiece", () -> Blackbox.isLoaded(), swerve).setNext(firePieceShoot, secondPieceFromFirst);
        TreeCommandNode root = shoot.clone().setNext(firstPiece);

        return new TreeAuto(root);
    }
}
