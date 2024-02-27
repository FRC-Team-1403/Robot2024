
package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private BooleanSupplier m_amp;
    private BooleanSupplier m_loading;
    private double m_fpga;
    private BooleanSupplier m_reset;
    private BooleanSupplier m_stageLine;
    private BooleanSupplier m_centerLine;

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

    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmSubsystem arm, Wrist wrist, 
            BooleanSupplier trigger, BooleanSupplier amp, BooleanSupplier loading, BooleanSupplier reset,
            BooleanSupplier stageLine, BooleanSupplier centerLine) {
        m_intakeAndShooter = intakeAndShooter;
        m_arm = arm;
        m_trigger = trigger;
        m_wrist = wrist;
        m_amp = amp;
        m_loading = loading;
        m_reset = reset;
        m_stageLine = stageLine;
        m_centerLine = centerLine;
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
                if(m_loading.getAsBoolean())
                {
                    m_arm.moveArm(Constants.Arm.kLoadingSetpoint);
                    m_wrist.setWristAngle(145);
                    m_state = State.LOADING_STATION;
                }
                if(m_intakeAndShooter.isShooterPhotogateTriggered() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    m_arm.moveArm(Constants.Arm.kDriveSetpoint);
                    m_intakeAndShooter.intakeStop();
                    // m_wrist.setWristAngle(115);
                    m_state = State.RAISE;
                }
                break;
            }
            case LOADING_STATION:
            {         
                if(m_arm.isAtSetpoint())
                {
                    m_wrist.setWristAngle(Constants.Wrist.kLoadingSetpoint);
                    m_intakeAndShooter.setShooterRPM(0.0);

                }
                if(m_intakeAndShooter.isShooterPhotogateTriggered())
                {
                    m_intakeAndShooter.intakeStop();
                    m_wrist.setWristAngle(145);
                    if(m_wrist.isAtSetpoint())
                    {
                        m_arm.moveArm(Constants.Arm.kDriveSetpoint);
                        m_state = State.RAISE;
                    }
                }
                if(m_reset.getAsBoolean()) {
                    m_state = State.RESET;
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
                break;
            }
            case LOADED:
            {
                if(m_amp.getAsBoolean())
                {
                    m_arm.moveArm(Constants.Arm.kAmpSetpoint);
                    m_wrist.setWristAngle(Constants.Wrist.kAmpSetpoint);
                    m_intakeAndShooter.setShooterRPM(2400);

                }
                else if(m_reset.getAsBoolean())
                {
                    m_arm.moveArm(Constants.Arm.kDriveSetpoint);
                    m_wrist.setWristAngle(Constants.Wrist.kDriveSetpoint);
                    m_intakeAndShooter.setShooterRPM(4800);
                } 
                else if(m_stageLine.getAsBoolean())
                {
                    m_arm.moveArm(124);
                    m_wrist.setWristAngle(SmartDashboard.getNumber("Wrist Angle - Stage", Constants.Wrist.kStageLineSetpoint));
                    m_intakeAndShooter.setShooterRPM(SmartDashboard.getNumber("Shooter RPM - Stage", Constants.IntakeAndShooter.kStageLineRPM));
                }
                else if(m_centerLine.getAsBoolean())
                {
                    m_wrist.setWristAngle(Constants.Wrist.kCenterLineSetpoint);
                    m_arm.moveArm(200);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kCenterLineRPM);
                }
                
                // TODO: add indicator for the driver/operator in case the robot is not ready to shoot
                if(m_trigger.getAsBoolean() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {                
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
                        m_state = State.RESET;
                }
                break;
        }     
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
