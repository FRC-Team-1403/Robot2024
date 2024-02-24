package team1403.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;

public class holdArmAutoCommand extends Command{
    private Wrist m_Wrist;
    private ArmSubsystem m_arm;
    private Timer m_timer;

    public holdArmAutoCommand(Wrist wrist, ArmSubsystem arm){
        m_Wrist = wrist;
        m_arm = arm;
    }

    @Override
    public void execute(){
        m_Wrist.setWristAngle(Constants.Wrist.kIntakeSetpoint);
        m_arm.moveArm(Constants.Arm.kIntakeSetpoint);
    }

    @Override
    public boolean isFinished(){
        return !(m_timer.get() >= 15);
    }

}
