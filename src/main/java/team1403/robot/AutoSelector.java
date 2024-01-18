package team1403.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    public static SendableChooser<Command> getAutonomousCommandChooser()  {
        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("Auto", new PathPlannerAuto("Auto"));
        autoChooser.addOption("cool", new PathPlannerAuto("kjsdk"));
        return autoChooser;
    }
    
}