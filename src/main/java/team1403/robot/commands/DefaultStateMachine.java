package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants.Setpoints;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;

public class DefaultStateMachine extends Command {

    private ArmSubsystem m_arm;
    private Wrist m_wrist;
    private BooleanSupplier m_triggerShot;
    private BooleanSupplier m_loadingStation;
    private BooleanSupplier m_resetToIntake;

    private boolean m_loadingStationSetpoint = false;

    enum State {
        LOWER,
        INTAKE,
        READY
    }

    private State m_state;

    public DefaultStateMachine(ArmSubsystem arm, Wrist wrist, BooleanSupplier triggerShot, BooleanSupplier loadingStation, BooleanSupplier resetToIntake) {
        m_arm = arm;
        m_wrist = wrist;
        m_triggerShot = triggerShot;
        m_state = State.LOWER;
    }

    @Override
    public void initialize() {
        if(Blackbox.getIsLoaded()) {
            m_state = State.READY;
        }
    }

    @Override
    public void execute() {
        switch(m_state) {
            case LOWER:
            {
                Setpoints.kLowerSetpoint.applySetpoint();
                Blackbox.setAutoTrigger(false);

                if(m_arm.isAtSetpoint() && m_wrist.isAtSetpoint()) {
                    m_state = State.INTAKE;
                }
                break;
            }
            case INTAKE:
            {
                if(m_loadingStation.getAsBoolean())
                    m_loadingStationSetpoint = true;
                else if(m_resetToIntake.getAsBoolean())
                    m_loadingStationSetpoint = false;

                if(m_loadingStationSetpoint)
                    Setpoints.kLoadingStationSetpoint.applySetpoint();
                else
                    Setpoints.kIntakeSetpoint.applySetpoint();

                Blackbox.setAutoTrigger(false);

                if(Blackbox.getIsLoaded()) {
                    m_state = State.READY;
                    m_loadingStationSetpoint = false;
                }

                break;
            }
            case READY:
            {

                Blackbox.getTargetSetpoint().applySetpoint();

                if(m_triggerShot.getAsBoolean() || Blackbox.getAutoTrigger()) {
                    Blackbox.setTrigger(true);
                    Blackbox.setAutoTrigger(false);
                }

                if(!Blackbox.getIsLoaded() || m_resetToIntake.getAsBoolean()) {
                    m_state = State.LOWER;
                }

                break;
            }
        }

        Logger.recordOutput("State Machine/State", m_state.toString());
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
