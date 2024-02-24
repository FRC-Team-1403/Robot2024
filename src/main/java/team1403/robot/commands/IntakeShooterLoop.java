package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;

public class IntakeShooterLoop extends Command {
    private IntakeAndShooter m_intakeAndShooter;
    private ArmSubsystem m_arm;
    private Wrist m_wrist;
    private BooleanSupplier m_trigger;

    private enum State
    {
        RESET,
        INTAKE,
        RAISE,
        LOAD, 
        LOADED,
        SHOOT
    }

    private State m_state;

    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmSubsystem arm, Wrist wrist, BooleanSupplier trigger) {
        m_intakeAndShooter = intakeAndShooter;
        m_arm = arm;
        m_trigger = trigger;
        m_wrist = wrist;
    }

    @Override
    public void initialize()
    {
        m_state = State.RESET;
    }


    @Override
    public void execute()
    {
        switch(m_state)
        {
            case RESET:
                m_arm.moveArm(Constants.Arm.kIntakeSetpoint);
                m_wrist.setWristAngle(Constants.Wrist.kIntakeSetpoint);
                m_intakeAndShooter.setIntakeSpeed(1.0);
                m_intakeAndShooter.setShooterSpeed(0.0);
                m_state = State.INTAKE;
                break;
            case INTAKE:
                if(m_intakeAndShooter.isIntakePhotogateTriggered())
                    m_intakeAndShooter.setIntakeSpeed(0.4);
                if(m_intakeAndShooter.isShooterPhotogateTriggered()) {
                    m_arm.moveArm(Constants.IntakeAndShooter.kShootingAngle);
                    m_wrist.setWristAngle(130);
                    m_state = State.RAISE;
                }
            break;
            case RAISE:
                if(m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    m_intakeAndShooter.setIntakeSpeed(-0.2);
                    m_state = State.LOAD;
                }
            break;
            case LOAD:
                if(!m_intakeAndShooter.isShooterPhotogateTriggered()) {
                    m_intakeAndShooter.setShooterSpeed(0.8);
                    m_intakeAndShooter.intakeStop();
                    m_state = State.LOADED;
                }
            break;
            case LOADED:
                //TODO: add indicator for the driver/operator in case the robot is not ready to shoot
                if(m_trigger.getAsBoolean() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {                
                    m_intakeAndShooter.setIntakeSpeed(0.5);
                    m_state = State.SHOOT;
                }
            break;
            case SHOOT:
                if(!m_intakeAndShooter.isIntakePhotogateTriggered())
                    m_state = State.RESET;
            break;
        }     
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
