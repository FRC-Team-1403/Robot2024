package team1403.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    private static SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("taxiAuto");

    public static void initAutoChooser() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static Command getSelectedAuto() {
        Command auto = autoChooser.getSelected();
        return auto;
    }

}