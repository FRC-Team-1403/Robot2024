package team1403.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    // private static SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("");

    public static void initAutoChooser() {
        // SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static Command getSelectedAuto() {
        // Command auto = autoChooser.getSelected();
        return AutoBuilder.buildAuto("No Auto");
    }

    
}
