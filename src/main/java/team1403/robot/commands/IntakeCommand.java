
package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.LED;
import team1403.robot.subsystems.LED.LEDState;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;

public class IntakeCommand extends Command {
    private IntakeAndShooter m_intakeAndShooter;
    private ArmSubsystem m_arm;
    private Wrist m_wrist;
    private double m_fpga;

    private enum State
    {
        RESET,
        LOWER,
        INTAKE,
        RAISE,
        LOAD, 
        LOADED,
        SHOOT,
        LOADING_STATION,
    }

    private State m_state;
    private boolean finished;

    public IntakeCommand(IntakeAndShooter intakeAndShooter, ArmSubsystem arm, Wrist wrist) {
        m_intakeAndShooter = intakeAndShooter;
        m_arm = arm;
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
        SmartDashboard.putString("State", m_state.toString());
        switch(m_state)
        {
            case RESET:
            {
                m_wrist.setWristAngle(145);
                m_intakeAndShooter.setIntakeSpeed(0.0);
                m_intakeAndShooter.setShooterRPM(0.0);
                if(m_wrist.isAtSetpoint())
                {
                    m_arm.moveArm(Constants.Arm.kIntakeSetpoint);
                    m_state = State.LOWER;
                }
                break;
            }
            case LOWER:
            {
                if(m_arm.isAtSetpoint())
                {
                    m_wrist.setWristAngle(Constants.Wrist.kIntakeSetpoint);
                    m_intakeAndShooter.setIntakeSpeed(1);
                    m_state = State.INTAKE;
                }
                break;
            }
            case INTAKE:
            {
                if(m_intakeAndShooter.isIntakePhotogateTriggered())
                    m_intakeAndShooter.setIntakeSpeed(0.7);
                if(m_intakeAndShooter.isShooterPhotogateTriggered() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    m_arm.moveArm(Constants.Arm.kDriveSetpoint);
                    m_intakeAndShooter.intakeStop();
                    // m_wrist.setWristAngle(115);
                    m_state = State.RAISE;
                }
                break;
            }
            case RAISE:
            {
                if(m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    m_wrist.setWristAngle(Constants.Wrist.kShootingAngle);
                    m_intakeAndShooter.setIntakeSpeed(-0.4);
                    m_state = State.LOAD;
                }
                break;
            }
            case LOAD:
            {
                if(!m_intakeAndShooter.isShooterPhotogateTriggered()) {
                    m_intakeAndShooter.intakeStop();
                    if(m_wrist.isAtSetpoint()) {
                        m_intakeAndShooter.setShooterRPM(4800);
                        m_state = State.LOADED;
                    }
                }
                finished = true;
                break;
            }
        }     
    }

    @Override
    public boolean isFinished()
    {
        return finished;
    }
}
