
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

public class ShootCommand extends Command {
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
    private boolean finished = false;

    public ShootCommand(IntakeAndShooter intakeAndShooter, ArmSubsystem arm, Wrist wrist) {
        m_intakeAndShooter = intakeAndShooter;
        m_arm = arm;
        m_wrist = wrist;
    }

    @Override
    public void initialize()
    {
        m_state = State.LOADED;
        finished = false;

    }


    @Override
    public void execute()
    {
        SmartDashboard.putString("State", m_state.toString());
        switch(m_state)
        {
            case LOADED:
            {
                     if(m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {                
                    m_intakeAndShooter.setIntakeSpeed(0.5);
                    m_state = State.SHOOT;
                    m_fpga = Timer.getFPGATimestamp();
                }
                break;
            }
            case SHOOT:       
                if(!m_intakeAndShooter.isIntakePhotogateTriggered() && !m_intakeAndShooter.isShooterPhotogateTriggered())
                {
                    if(Timer.getFPGATimestamp() - m_fpga > 0.1)
                        finished = true;
                }

                break;
        }     
    }

    @Override
    public boolean isFinished()
    {
        return finished;
    }
}