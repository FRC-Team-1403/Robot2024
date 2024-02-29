
package team1403.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.LED;
import team1403.robot.subsystems.LED.LEDState;
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
    private BooleanSupplier m_resetToNeutral;
    private BooleanSupplier m_stageLine;
    private BooleanSupplier m_centerLine;
    private LED m_led;
    private BooleanSupplier m_resetToIntake;
    private BooleanSupplier m_launchpad;
    private DoubleSupplier m_expel;
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

    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmSubsystem arm, Wrist wrist, LED led,
            BooleanSupplier trigger, BooleanSupplier amp, BooleanSupplier loading, BooleanSupplier resetToIntake,
            BooleanSupplier stageLine, BooleanSupplier centerLine, BooleanSupplier resetToNeutral, BooleanSupplier launchpad,
            DoubleSupplier expel) {
        m_intakeAndShooter = intakeAndShooter;
        m_arm = arm;
        m_trigger = trigger;
        m_wrist = wrist;
        m_amp =  amp;
        m_loading =  loading;
        m_resetToNeutral =  resetToNeutral;
        m_stageLine =  stageLine;
        m_centerLine =  centerLine;
        m_led = led;
        m_resetToIntake =  resetToIntake;
        m_launchpad = launchpad;
        m_expel = expel;
    }

    @Override
    public void initialize()
    {
        // if(Constants.Auto.kisIntaked)
        //     m_state = State.RAISE;
        // else
        m_state = State.RESET;
        Constants.Auto.kFinished = false;
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
                if(m_intakeAndShooter.isIntakePhotogateTriggered() && !Constants.Auto.kisIntaked)
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
                if(m_resetToIntake.getAsBoolean() || m_resetToIntake.getAsBoolean()) {
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
                        if(m_expel.getAsDouble() >= Constants.IntakeAndShooter.kExpelDeadzone){
                            m_intakeAndShooter.setIntakeSpeed(-0.7);
                            m_intakeAndShooter.setShooterRPM(-1500);
                        }
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
                if(m_amp.getAsBoolean())
                {
                    m_arm.moveArm(Constants.Arm.kAmpSetpoint);
                    m_wrist.setWristAngle(Constants.Wrist.kAmpSetpoint);
                    m_intakeAndShooter.setShooterRPM(2400);

                }
                else if(m_resetToNeutral.getAsBoolean())
                {
                    m_arm.moveArm(Constants.Arm.kDriveSetpoint);
                    m_wrist.setWristAngle(Constants.Wrist.kDriveSetpoint);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kCloseRPM);
                } 
                else if(m_launchpad.getAsBoolean())
                {
                    m_arm.moveArm(124);
                    m_wrist.setWristAngle(Constants.Wrist.kLaunchpadSetpoint);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kLaunchpadRPM);
                } else if(m_stageLine.getAsBoolean()){
                    m_arm.moveArm(124);
                    m_wrist.setWristAngle(Constants.Wrist.kStageLineSetpoint);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kStageLineRPM);
                }
                else if(m_centerLine.getAsBoolean())
                {
                    m_wrist.setWristAngle(Constants.Wrist.kCenterLineSetpoint);
                    m_arm.moveArm(200);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kCenterLineRPM);
                }
                else if(m_resetToIntake.getAsBoolean()) {
                    m_state = State.RESET;
                }
                
                // TODO: add indicator for the driver/operator in case the robot is not ready to shoot
                if(m_trigger.getAsBoolean() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {        
                    if(Timer.getFPGATimestamp() - m_fpga > 1){
                             
                        m_intakeAndShooter.setIntakeSpeed(0.5);
                        m_fpga = Timer.getFPGATimestamp(); 
                        m_state = State.SHOOT;
                    }

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
                    if(Timer.getFPGATimestamp() - m_fpga > 0.1)
                        if (Constants.Auto.kInAuto) m_state = State.AUTOOVER;
                        else m_state = State.RESET;
                }
                break;

            case AUTOOVER:
            {
                Constants.Auto.kisIntaked = false;
                Constants.Auto.kFinished = true;
                Constants.Auto.kStageLine = false;
                Constants.Auto.kDriveSetpoint = false;
                break;
            }
        }     
    }

    @Override
    public boolean isFinished()
    {
        return Constants.Auto.kFinished;
    }
}
