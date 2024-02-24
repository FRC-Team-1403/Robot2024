package team1403.robot.commands;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants.*;
import team1403.robot.subsystems.arm.ArmSubsystem;


public class RunIntakeShooterAuto extends Command {
    private team1403.robot.subsystems.IntakeAndShooter m_intake;
    private boolean done = false;
    private team1403.robot.subsystems.arm.Wrist m_wrist;
    private ArmSubsystem m_arm;

    public RunIntakeShooterAuto(team1403.robot.subsystems.IntakeAndShooter intake,team1403.robot.subsystems.arm.Wrist  wrist, ArmSubsystem arm){
        m_intake = intake;
        m_wrist = wrist;
        m_arm = arm;
    }

    @Override
    public void initialize(){
        done = false;
    }

    @Override
    public void execute(){
        m_wrist.setWristAngle(140);
        Timer.delay(0.15);
        m_arm.moveArm(Arm.kIntakeSetpoint);
        m_wrist.setWristAngle(Wrist.kIntakeSetpoint);
            if(m_wrist.isAtSetpoint() && m_arm.isAtSetpoint())
            {
                m_intake.setShooterSpeed(0.6);
                Timer.delay(1.5);
                m_intake.setIntakeSpeed(1);
                Timer.delay(1);
                done = true;
            }
        }
    

    
    @Override
    public boolean isFinished(){
        m_intake.setIntakeSpeed(0);
        m_intake.setShooterSpeed(0);
        return done;
    }
}
