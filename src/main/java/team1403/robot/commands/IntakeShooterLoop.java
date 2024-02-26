
package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.IntakeAndShooter.Check;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;

public class IntakeShooterLoop extends Command {
    private IntakeAndShooter m_intakeAndShooter;
    private ArmSubsystem m_arm;
    private Wrist m_wrist;
    private BooleanSupplier m_trigger;
    private BooleanSupplier m_amp;

    private enum State
    {
        RESET,
        LOWER,
        INTAKE,
        RAISE,
        LOAD, 
        LOADED,
        SHOOT
    }

    private State m_state;

    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmSubsystem arm, Wrist wrist, BooleanSupplier trigger, BooleanSupplier amp) {
        m_intakeAndShooter = intakeAndShooter;
        m_arm = arm;
        m_trigger = trigger;
        m_wrist = wrist;
        m_amp = amp;
    }

    @Override
    public void initialize()
    {
        m_state = State.RESET;
    }


    @Override
    public void execute()
    {
        SmartDashboard.putString("State", m_state.toString());
        switch(m_state)
        {
            case RESET:
                m_wrist.setWristAngle(140);
                m_intakeAndShooter.setIntakeSpeed(0.0);
                m_intakeAndShooter.setShooterSpeed(0.0);
                if(m_wrist.isAtSetpoint())
                {
                    m_arm.moveArm(Constants.Arm.kIntakeSetpoint);
                    m_state = State.LOWER;
                }
                break;
            case LOWER:
                if(m_arm.isAtSetpoint())
                {
                    m_wrist.setWristAngle(Constants.Wrist.kIntakeSetpoint);
                    m_intakeAndShooter.setIntakeSpeed(1.0);
                    m_state = State.INTAKE;
                }
                break;
            case INTAKE:
                if(m_intakeAndShooter.isIntakePhotogateTriggered())
                    m_intakeAndShooter.setIntakeSpeed(0.4);
                if(m_intakeAndShooter.isShooterPhotogateTriggered() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    m_arm.moveArm(107);
                    m_intakeAndShooter.shooterStop();
                    //m_wrist.setWristAngle(130);
                    m_state = State.RAISE;
                }
                break;
            case RAISE:
                if(m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    m_wrist.setWristAngle(Constants.IntakeAndShooter.kShootingAngle);
                    m_intakeAndShooter.setIntakeSpeed(-0.3);
                    m_state = State.LOAD;
                }
                break;
            case LOAD:
                if(m_intakeAndShooter.isShooterPhotogateCheckTrigger() == Check.FALSE) {
                    m_intakeAndShooter.intakeStop();
                    if(m_wrist.isAtSetpoint()) {
                        m_intakeAndShooter.setShooterSpeed(0.8);
                        m_state = State.LOADED;
                    }
                }
                else if (m_intakeAndShooter.isShooterPhotogateCheckTrigger() == Check.MAYBE) {
                    m_intakeAndShooter.setIntakeSpeed(.5);
                }else {
                    m_intakeAndShooter.setIntakeSpeed(1);
                }
                break;
            case LOADED:
                if(m_amp.getAsBoolean())
                {
                    m_arm.moveArm(Constants.Arm.kAmpSetpoint);
                    m_wrist.setWristAngle(Constants.Wrist.kAmpSetpoint);
                    m_intakeAndShooter.setShooterSpeed(0.6);
                }
                //TODO: add indicator for the driver/operator in case the robot is not ready to shoot
                if(m_trigger.getAsBoolean() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {                
                    m_intakeAndShooter.setIntakeSpeed(0.5);
                    m_state = State.SHOOT;
                }
                break;
            case SHOOT:       
                if(m_intakeAndShooter.isIntakePhotogateCheckTrigger() == Check.FALSE)
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
