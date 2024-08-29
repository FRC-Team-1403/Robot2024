package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.IntakeAndShooter;

public class DefaultIntakeCommand extends Command {


    private enum IntakeState {
        UNLOADED,
        BACKDRIVE, //move the note backward to remove it from shooter photogate
        LOADED
    }

    private IntakeState m_state = IntakeState.UNLOADED;
    private IntakeAndShooter m_intakeSubsystem;
    private BooleanSupplier m_ejectSupplier;
    private double m_intakeSpeed = 0;
    private double m_shooterRPM = 0;
    private int m_counter = 0;

    public DefaultIntakeCommand(IntakeAndShooter intakeSubsystem, BooleanSupplier ejectSupplier) {
        m_intakeSubsystem = intakeSubsystem;
        m_ejectSupplier = ejectSupplier;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        if(m_intakeSubsystem.isIntakePhotogateTriggered())
        {
            if(m_intakeSubsystem.isShooterPhotogateTriggered())
                m_state = IntakeState.BACKDRIVE;
            else
                m_state = IntakeState.LOADED;
        }
    }

    @Override
    public void execute() {
        switch(m_state) {
            case UNLOADED:
            {
                m_intakeSpeed = 1;
                m_shooterRPM = 0;
                if (m_intakeSubsystem.isIntakePhotogateTriggered() && m_intakeSubsystem.isShooterPhotogateTriggered()) {
                    m_intakeSpeed = 0;
                    m_state = IntakeState.BACKDRIVE;
                }
                break;
            }
            case BACKDRIVE:
            {
                m_intakeSpeed = -0.4;
                if(m_intakeSubsystem.isIntakePhotogateTriggered() && !m_intakeSubsystem.isShooterPhotogateTriggered()) {
                    m_intakeSpeed = 0;
                    m_counter = 0;
                    m_state = IntakeState.LOADED;
                }

                if(!m_intakeSubsystem.isIntakePhotogateTriggered() && !m_intakeSubsystem.isShooterPhotogateTriggered()) {
                    m_counter++;
                    //check for persistence
                    if(m_counter >= 3) {
                        m_state = IntakeState.UNLOADED;
                        m_counter = 0;
                    }
                }
                else {
                    m_counter = 0;
                }
                break;
            }
            case LOADED:
            {
                m_intakeSpeed = 0;

                if(!m_intakeSubsystem.isIntakePhotogateTriggered() && !m_intakeSubsystem.isShooterPhotogateTriggered()) {
                    m_counter++;
                    //check for persistence
                    if(m_counter >= 3) {
                        m_state = IntakeState.UNLOADED;
                        m_counter = 0;
                    }
                }
                else {
                    m_counter = 0;
                }
                break;
            }
        }

        if(m_ejectSupplier.getAsBoolean()) {
            m_intakeSpeed = -1;
            m_shooterRPM = -1500;
        }

        m_intakeSubsystem.setIntakeSpeed(m_intakeSpeed);
        m_intakeSubsystem.setShooterRPM(m_shooterRPM);

        Logger.recordOutput("Intake/IntakeState", m_state.toString());
    }


    @Override
    public boolean isFinished() {
        return false;
    }    
}
