package team1403.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.Constants.Intake;
import team1403.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {
    private IntakeSubsystem m_intake;

    public ShootCommand(IntakeSubsystem intake) {
        m_intake = intake;
    }

    @Override 
    public boolean isFinished() {
       if (m_intake.isShooterGateOn()) {
         new SequentialCommandGroup(    
            new WaitCommand(1),
            new InstantCommand(() -> m_intake.setEverythingSpeed(0)));
        return true;
       }
        return false;
        
    }

    @Override
    public void execute() {
        m_intake.setIntakeSpeed(0.1);
    }
}