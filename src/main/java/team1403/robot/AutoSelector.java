package team1403.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    //Todo update and fix this
    public static SendableChooser<Command> getAutonomousCommandChooser()  {
        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("Auto", new PathPlannerAuto("Auto"));
        autoChooser.addOption("Amp Optimal", new PathPlannerAuto("Amp Optimal"));
        autoChooser.addOption("realistic auton", new PathPlannerAuto("realistic auton"));
        autoChooser.addOption("source optimal", new PathPlannerAuto("source optimal"));
        autoChooser.addOption("super hard auton", new PathPlannerAuto("super hard auton"));
        autoChooser.addOption("taxi path", new PathPlannerAuto("taxi"));
        return autoChooser;
    }
    
}