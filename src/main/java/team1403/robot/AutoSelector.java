package team1403.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    public Command getAutonomousCommand() {
            return new PathPlannerAuto("Example Auto");
    }
}