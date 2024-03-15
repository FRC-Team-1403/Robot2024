
package team1403.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private BooleanSupplier m_ampShooting;
    private CommandXboxController m_ops;
    private int m_counter;


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

    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmSubsystem arm, Wrist wrist, LED led, CommandXboxController ops,
            BooleanSupplier trigger, BooleanSupplier amp, BooleanSupplier loading, BooleanSupplier resetToIntake,
            BooleanSupplier stageLine, BooleanSupplier centerLine, BooleanSupplier resetToNeutral, BooleanSupplier launchpad,
            DoubleSupplier expel,BooleanSupplier ampShooting) {
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
        m_ampShooting = ampShooting;
        m_ops = ops;
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

    private double deadband(double input)
    {
        if(Math.abs(input) <= 0.05)
            return 0;
        return input;
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
                    m_wrist.setWristAngle(140);
                    m_state = State.LOADING_STATION;
                }
                if(m_intakeAndShooter.isShooterPhotogateTriggered() && m_intakeAndShooter.isIntakePhotogateTriggered() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    m_arm.moveArm(Constants.Arm.kDriveSetpoint);
                    m_intakeAndShooter.intakeStop();
                    // m_wrist.setWristAngle(115);
                    m_state = State.RAISE;
                }
                m_arm.moveArm(m_arm.getPivotAngleSetpoint() - deadband(m_ops.getRightY()));
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
                if(m_resetToIntake.getAsBoolean() || m_resetToIntake.getAsBoolean()) {
                    m_state = State.RESET;
                }
                break;
            }
            case RAISE:
            {
                if(m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    m_wrist.setWristAngle(Constants.Wrist.kShootingAngle);

                    if(m_stageLine.getAsBoolean()){
                        m_arm.moveArm(124);
                        m_wrist.setWristAngle(Constants.Wrist.kStageLineSetpoint);
                        m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kStageLineRPM);
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
                        m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kCloseRPM);
                            m_fpga = Timer.getFPGATimestamp(); 
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
                else if(m_resetToNeutral.getAsBoolean())
                {
                    m_arm.moveArm(Constants.Arm.kDriveSetpoint);
                    m_wrist.setWristAngle(Constants.Wrist.kShootingAngle);
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
                    m_arm.moveArm(130);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kCenterLineRPM);
                } else if(m_ampShooting.getAsBoolean()){
                    m_wrist.setWristAngle(Constants.Wrist.kAmpShoootingSetpoint);
                    m_arm.moveArm(124);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kStageLineRPM);
                }
                
                // TODO: add indicator for the driver/operator in case the robot is not ready to shoot
                if(m_trigger.getAsBoolean() && m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {        
                    if(m_intakeAndShooter.teleopIsReady()){
                        m_intakeAndShooter.setIntakeSpeed(0.5);
                        m_fpga = Timer.getFPGATimestamp(); 
                        m_state = State.SHOOT;
                    }
                }

                if(!m_intakeAndShooter.isIntakePhotogateTriggered())
                {
                    if(m_counter >= 5) {
                        m_state = State.RESET;
                        m_counter = 0;
                    }
                    else m_counter++;
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
                    //if (Constants.Auto.kInAuto) m_state = State.AUTOOVER;
                    if(Timer.getFPGATimestamp() - m_fpga > 0.1)
                        m_state = State.RESET;
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
        if(m_expel.getAsDouble() >= Constants.IntakeAndShooter.kExpelDeadzone){
            m_intakeAndShooter.setIntakeSpeed(-0.7);
            m_intakeAndShooter.setShooterRPM(-1500);
            // get approval 
            // m_state = State.RESET;
        }
        if(m_resetToIntake.getAsBoolean()) {
            m_state = State.RESET;
        }
        Logger.recordOutput("State", m_state.toString());
    }

    @Override
    public boolean isFinished()
    {
        return Constants.Auto.kFinished;
    }
}
