
package team1403.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Logged;
import team1403.lib.util.CougarLogged;
import team1403.robot.Constants;
import team1403.robot.Constants.Setpoints;
import team1403.robot.subsystems.ArmWristSubsystem;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.subsystems.LED;
import team1403.robot.subsystems.SonicBlasterSetpoint;
import team1403.robot.subsystems.LED.LEDState;

public class IntakeShooterLoop extends Command implements CougarLogged {
    private IntakeAndShooter m_intakeAndShooter;
    private ArmWristSubsystem m_armwrist;
    private BooleanSupplier m_trigger;
    private BooleanSupplier m_loading;
    private double m_fpga;
    private LED m_led;
    private BooleanSupplier m_resetToIntake;
    private DoubleSupplier m_expel;
    private XboxController m_ops;
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
        LOADING_STATION
    }

    private State m_state = State.RESET;

    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmWristSubsystem armwrist, LED led, XboxController ops,
            BooleanSupplier trigger, BooleanSupplier loading, BooleanSupplier resetToIntake,
            DoubleSupplier expel) {
        m_intakeAndShooter = intakeAndShooter;
        m_trigger = trigger;
        m_armwrist = armwrist;
        m_loading =  loading;
        m_led = led;
        m_resetToIntake =  resetToIntake;
        m_expel = expel;
        m_ops = ops;

        Constants.kDriverTab.addString("State", () -> m_state.toString());

        addRequirements(m_armwrist, m_led, m_intakeAndShooter);
    }

    @Override
    public void initialize()
    {
        // if(Constants.Auto.kisIntaked)
        //     m_state = State.RAISE;
        // else
        m_state = State.RESET;
        // time.reset();
        // time.start();   
        
    }

    private void applySetpoint(SonicBlasterSetpoint setpoint) {
        m_intakeAndShooter.applySetpoint(setpoint);
        m_armwrist.applySetpoint(setpoint);
    }

    @Override
    public void execute()
    {
        switch(m_state)
        {
            case RESET:
            {
                m_led.setLedColor(0.41);
                m_armwrist.setWristSetpoint(140);
                m_intakeAndShooter.setIntakeSpeed(0.0);
                m_intakeAndShooter.setShooterRPM(0.0);
                if(m_armwrist.isWristAtSetpoint())
                {
                    m_armwrist.setArmSetpoint(Constants.Arm.kIntakeSetpoint);
                    m_state = State.LOWER;
                }
                break;
            }
            case LOWER:
            {
                if(m_armwrist.isArmAtSetpoint())
                {
                    m_armwrist.setWristSetpoint(Constants.Wrist.kIntakeSetpoint);
                    m_intakeAndShooter.setIntakeSpeed(1);
                    m_state = State.INTAKE;
                }
                break;
            }
            case INTAKE:
            {
                // if(m_intakeAndShooter.isIntakePhotogateTriggered() && !Constants.Auto.kisIntaked) // no need to reduce the speed most likely
                //     m_intakeAndShooter.setIntakeSpeed(0.7);
                if(m_loading.getAsBoolean())
                {
                    m_armwrist.setArmSetpoint(Constants.Arm.kLoadingSetpoint);
                    m_armwrist.setWristSetpoint(140);
                    m_state = State.LOADING_STATION;
                }
                if(m_intakeAndShooter.isShooterPhotogateTriggered() && m_intakeAndShooter.isIntakePhotogateTriggered() && m_armwrist.isArmAtSetpoint() && m_armwrist.isWristAtSetpoint()) {
                    m_armwrist.setArmSetpoint(Constants.Arm.kDriveSetpoint);
                    m_intakeAndShooter.intakeStop();
                    // m_armwrist.setWristSetpoint(115);
                    m_state = State.RAISE;
                }
                m_armwrist.setArmSetpoint(m_armwrist.getPivotAngle() - MathUtil.applyDeadband(m_ops.getRightY(), 0.05));
                break;
            }
            case LOADING_STATION:
            {
                if(m_armwrist.isArmAtSetpoint())
                {
                    m_armwrist.setWristSetpoint(Constants.Wrist.kLoadingSetpoint);
                    m_intakeAndShooter.setShooterRPM(0.0);

                }
                if(m_intakeAndShooter.isShooterPhotogateTriggered())
                {
                    m_intakeAndShooter.intakeStop();
                    m_armwrist.setWristSetpoint(140);
                    if(m_armwrist.isArmAtSetpoint())
                    {
                        m_armwrist.setArmSetpoint(Constants.Arm.kDriveSetpoint);
                        m_state = State.RAISE;
                    }
                }
                if(m_resetToIntake.getAsBoolean()) {
                    m_state = State.RESET;
                }
                break;
            }
            case RAISE:
            {
                if(m_armwrist.isArmAtSetpoint() && m_armwrist.isWristAtSetpoint()) {
                    m_armwrist.applySetpoint(Blackbox.requestedSetpoint);
                    m_intakeAndShooter.setIntakeSpeed(-0.4);
                    m_state = State.LOAD;
                }
                break;
            }
            case LOAD:
            {
                if(!m_intakeAndShooter.isShooterPhotogateTriggered()) {
                    m_intakeAndShooter.applySetpoint(Blackbox.requestedSetpoint);
                    if(m_armwrist.isWristAtSetpoint()) {
                        m_fpga = Timer.getFPGATimestamp(); 
                        m_state = State.LOADED;
                    }
                }
                break;
            }
            case LOADED:
            {
                applySetpoint(Blackbox.requestedSetpoint);
                /*
                if(m_amp.getAsBoolean())
                {
                    m_armwrist.setArmSetpoint(Constants.Arm.kAmpSetpoint);
                    m_armwrist.setWristSetpoint(Constants.Wrist.kAmpSetpoint);
                    m_intakeAndShooter.setShooterRPM(2400);
                }
                else if(m_resetToNeutral.getAsBoolean())
                {
                    applySetpoint(Setpoints.kDriveSetpoint);
                } 
                else if(m_launchpad.getAsBoolean())
                {
                    m_armwrist.setArmSetpoint(124);
                    m_armwrist.setWristSetpoint(Constants.Wrist.kLaunchpadSetpoint);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kLaunchpadRPM);
                } else if(m_stageLine.getAsBoolean()){
                    applySetpoint(Setpoints.kStageSetpoint);
                }
                else if(m_centerLine.getAsBoolean())
                {
                    m_armwrist.setWristSetpoint(Constants.Wrist.kCenterLineSetpoint);
                    m_armwrist.setArmSetpoint(130);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kCenterLineRPM);
                } else if(m_ampShooting.getAsBoolean()){
                    m_armwrist.setWristSetpoint(Constants.Wrist.kAmpShoootingSetpoint);
                    m_armwrist.setArmSetpoint(124);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kStageLineRPM);
                } else if(m_feedShot.getAsBoolean()) {
                    m_armwrist.setWristSetpoint(Constants.Wrist.kShootingAngle + 5);
                    m_armwrist.setArmSetpoint(Constants.Arm.kDriveSetpoint);
                    m_intakeAndShooter.setShooterRPM(Constants.IntakeAndShooter.kFeedShotRPM);
                }*/

                if(m_armwrist.isArmAtSetpoint() && m_armwrist.isWristAtSetpoint() && m_intakeAndShooter.isIntakePhotogateTriggered() && m_intakeAndShooter.teleopIsReady()) {
                    if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Blue) m_led.setLedMode(LEDState.DARK_BLUE);
                    else m_led.setLedMode(LEDState.DARK_RED);
                }
                else {
                    m_led.setLedMode(LEDState.YELLOW_FLASH);
                }

                // TODO: add indicator for the driver/operator in case the robot is not ready to shoot
                if(m_trigger.getAsBoolean() && m_armwrist.isArmAtSetpoint() && m_armwrist.isWristAtSetpoint()) {
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

                break;
            }
            case SHOOT:
            {
                if(!m_intakeAndShooter.isIntakePhotogateTriggered() && !m_intakeAndShooter.isShooterPhotogateTriggered())
                {
                    //if (Constants.Auto.kInAuto) m_state = State.AUTOOVER;
                    if(Timer.getFPGATimestamp() - m_fpga > 0.1) {
                        m_state = State.RESET;
                        Blackbox.requestedSetpoint = Setpoints.kDriveSetpoint;
                    }
                }
                break;
            }
        }
        if(m_expel.getAsDouble() >= Constants.IntakeAndShooter.kExpelDeadzone){
            m_intakeAndShooter.setIntakeSpeed(-1);
            m_intakeAndShooter.setShooterRPM(-1500);
            // get approval
            // m_state = State.RESET;
        }
        if(m_resetToIntake.getAsBoolean()) {
            m_state = State.RESET;
        }
        log("State", m_state.toString());
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
