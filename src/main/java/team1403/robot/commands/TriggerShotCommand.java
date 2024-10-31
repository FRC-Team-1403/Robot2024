package team1403.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Blackbox;

public class TriggerShotCommand extends Command {
    
    Timer timer = new Timer();

    @Override
    public void initialize() {
        Blackbox.setTrigger(true);
        timer.restart();
    }

    @Override
    public boolean isFinished() {
        if(timer.hasElapsed(2)) Blackbox.setTrigger(false);
        
        return !Blackbox.getTrigger();
    }

}
