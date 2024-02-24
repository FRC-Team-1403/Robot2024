package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;

public class CombinedShootCommand extends Command {
    IntakeAndShooter m_intakeAndShooter;
    ArmSubsystem m_arm;
    Wrist m_wrist;

    /* assumes that we have a piece loaded */
    CombinedShootCommand(IntakeAndShooter intakeAndShooter, ArmSubsystem arm, Wrist wrist) {
        m_intakeAndShooter = intakeAndShooter;
        m_arm = arm;
        m_wrist = wrist;
    }

    @Override
    public void initialize()
    {
        m_arm.moveArm(Constants.IntakeAndShooter.kShootingAngle);
    }


    @Override
    public void execute()
    {
        if(!m_arm.isAtSetpoint())
            return;

        m_wrist.setWristAngle(130);

        if(!m_wrist.isAtSetpoint())
            return;

        m_intakeAndShooter.setIntakeSpeed(-0.2);

        if(m_intakeAndShooter.isShooterPhotogateTriggered())
            return;

        m_intakeAndShooter.setIntakeSpeed(0);
        
    }

    @Override
    public boolean isFinished()
    {
        return m_arm.isAtSetpoint() && m_wrist.isAtSetpoint() && !m_intakeAndShooter.isShooterPhotogateTriggered();
    }

    
}
