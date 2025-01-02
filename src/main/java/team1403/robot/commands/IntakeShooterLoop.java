
package team1403.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.lib.util.CougarLogged;


public class IntakeShooterLoop extends Command implements CougarLogged {
    
    private IntakeAndShooter m_IntakeAndShooter;
    
    
    
    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter){
        m_IntakeAndShooter = intakeAndShooter;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_IntakeAndShooter.setIntakeMotor(0.5, m_IntakeAndShooter.m_intakeWheels);

        if (m_IntakeAndShooter.isPhotoGateTriggered(m_IntakeAndShooter.m_photogateShooter)) {
            m_IntakeAndShooter.setIntakeMotor(-0.5, m_IntakeAndShooter.m_intakeWheels);
        }

        if(!m_IntakeAndShooter.isPhotoGateTriggered(m_IntakeAndShooter.m_photogateShooter) && m_IntakeAndShooter.isPhotoGateTriggered(m_IntakeAndShooter.m_photogateIntake)) {
            m_IntakeAndShooter.setIntakeMotor(0, m_IntakeAndShooter.m_intakeWheels);
        }

        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
