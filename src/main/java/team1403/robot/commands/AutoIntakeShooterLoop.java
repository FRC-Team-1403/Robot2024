
package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.LED;
import team1403.robot.subsystems.LED.LEDState;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;

public class AutoIntakeShooterLoop extends Command {
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
    private LED m_led;
    private BooleanSupplier m_resetToReset;
    private BooleanSupplier m_closeSideShoot;
    private boolean m_side;
    private Timer time;
    private boolean m_secondSourceShoot;
    private int m_count;
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
        AUTOOVER
    }

    private State m_state;

    public AutoIntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmSubsystem arm, Wrist wrist, LED led, 
            BooleanSupplier stageLine, BooleanSupplier centerLine, boolean side, BooleanSupplier closeSideShoot,
            boolean secondSourceShoot) {
        m_intakeAndShooter = intakeAndShooter;
        m_arm = arm;
        m_trigger = () -> Blackbox.getTrigger();
        m_wrist = wrist;
        m_amp =  () -> false;
        m_loading =  () -> false;
        m_reset =  () -> false;
        m_stageLine =  stageLine;
        m_centerLine =  centerLine;
        m_led = led;
        m_resetToReset =  () -> false;
        m_side = side;
        m_closeSideShoot = closeSideShoot;
        m_secondSourceShoot = secondSourceShoot;
        Blackbox.setAutoFinished(false);
    }

    @Override
    public void initialize()
    {
        // if(Constants.Auto.kisIntaked)
        //     m_state = State.RAISE;
        // else
        m_state = State.RESET;
        m_count = 0;
        Blackbox.setAutoFinished(false);
        if(m_intakeAndShooter.isIntakePhotogateTriggered())
        {
            m_arm.moveArm(118);
            m_wrist.setWristAngle(129);
            if(!m_intakeAndShooter.isShooterPhotogateTriggered())
            {
                m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kCloseRPM);
                m_arm.moveArm(Constants.Arm.kDriveSetpoint);
                m_state = State.RAISE;
            }
        }
        // time.reset();
        // time.start();
    }


    @Override
    public void execute()
    {
        SmartDashboard.putString("State", m_state.toString());
        switch(m_state)
        {
            case RESET:
            {
                m_wrist.setWristAngle(140);
                m_intakeAndShooter.setIntakeSpeed(1.0);
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
                // if(m_intakeAndShooter.isIntakePhotogateTriggered())
                //     m_intakeAndShooter.setIntakeSpeed(0.7);
                if(m_loading.getAsBoolean())
                {
                    m_arm.moveArm(Constants.Arm.kLoadingSetpoint);
                    m_wrist.setWristAngle(140);
                    m_state = State.LOADING_STATION;
                }
                if(m_intakeAndShooter.isShooterPhotogateTriggered() && m_intakeAndShooter.isIntakePhotogateTriggered() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    m_arm.moveArm(Constants.Arm.kDriveSetpoint);
                    m_intakeAndShooter.intakeStop();
                    // m_wrist.setWristAngle(115);
                    m_state = State.RAISE;
                }
                if(m_trigger.getAsBoolean())
                {
                    m_count++;
                    if(m_count > 10)
                    {
                        Blackbox.setAutoFinished(true);
                        Blackbox.setTrigger(false);
                        System.out.println("Skipped gamepiece");
                    }
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
                    m_wrist.setWristAngle(140);
                    if(m_wrist.isAtSetpoint())
                    {
                        m_arm.moveArm(Constants.Arm.kDriveSetpoint);
                        m_state = State.RAISE;
                    }
                }
                if(m_reset.getAsBoolean() || m_resetToReset.getAsBoolean()) {
                    m_state = State.RESET;
                }
                break;
            }
            case RAISE:
            {
                if(m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    if (m_side && !m_stageLine.getAsBoolean()) m_wrist.setWristAngle(Constants.Wrist.kShootingAngle + 5);
                    else if (!m_side && !m_stageLine.getAsBoolean()) m_wrist.setWristAngle(Constants.Wrist.kShootingAngle + 3);
                    
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
                        if (m_side) m_wrist.setWristAngle(Constants.Wrist.kStageLineSideSetpoint -2);
                        else if (m_secondSourceShoot) m_wrist.setWristAngle(Constants.Wrist.kStageLineSetpoint+1);
                        else m_wrist.setWristAngle(Constants.Wrist.kStageLineSetpoint + 2);
                        m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kStageLineRPM);
                    } 
                    else if(m_centerLine.getAsBoolean())
                    {
                        m_wrist.setWristAngle(Constants.Wrist.kCenterLineSetpoint);
                        m_arm.moveArm(200);
                        m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kCenterLineRPM);
                    }
                    else if(m_resetToReset.getAsBoolean()) {
                        m_state = State.RESET;
                    }
                    else if(m_closeSideShoot.getAsBoolean()) {
                        m_wrist.setWristAngle(Constants.Wrist.kDefaultClose + 20);
                    }


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
                        if(m_intakeAndShooter.getShooterRPMSetpoint() == 0)
                            m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kCloseRPM);
                        if(m_intakeAndShooter.isReady()){
                            m_fpga = Timer.getFPGATimestamp(); 
                            m_state = State.LOADED;
                        }
                    }
                }
                break;
            }
            case LOADED:
            {
                
                // if(Constants.Auto.kInAuto && !m_trigger.getAsBoolean()) {
                //     m_state = State.AUTOOVER;
                // }
                
                // TODO: add indicator for the driver/operator in case the robot is not ready to shoot
                if(m_trigger.getAsBoolean() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint() && m_intakeAndShooter.isReady()) {                     
                    m_intakeAndShooter.setIntakeSpeed(0.5);
                    m_fpga = Timer.getFPGATimestamp(); 
                    m_state = State.SHOOT;
                }   
                
                // if(m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()){
                //     m_led.setLedMode(LEDState.YELLOW);
                // } else{
                //     m_led.setLedMode(LEDState.OFF);
                // }

                break;
            }
            case SHOOT:       
                if(!m_intakeAndShooter.isIntakePhotogateTriggered() && !m_intakeAndShooter.isShooterPhotogateTriggered())
                {
                    if(Timer.getFPGATimestamp() - m_fpga > 0.1) {
                        if (DriverStation.isAutonomous()) m_state = State.AUTOOVER;
                        else m_state = State.RESET;
                        Blackbox.setTrigger(false);
                        m_intakeAndShooter.setIntakeSpeed(1.0);
                    }
                }
                break;

            case AUTOOVER:
            {
                Blackbox.setAutoFinished(true);
                break;
            }
        }  
        Logger.recordOutput("Auto State", m_state.toString());
    }

    @Override
    public boolean isFinished()
    {
        return Blackbox.getAutoFinished();
    }
}
