package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Blackbox;

public class TriggerShotCommand extends Command {
    
    @Override
    public void initialize() {
        Blackbox.setTrigger(true);
    }

    @Override
    public boolean isFinished() {
        return !Blackbox.getTrigger();
    }

}
