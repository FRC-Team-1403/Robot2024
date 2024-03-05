package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeAndShooter;

public class TestRunIntakeAndShooter extends Command {
    private IntakeAndShooter m_IntakeAndShooter;
    private boolean m_intaked;
    
    
    public TestRunIntakeAndShooter(IntakeAndShooter intakeAndShooter, boolean intaked) {
        m_IntakeAndShooter = intakeAndShooter;
        m_intaked = intaked;
    }

    @Override
    public void initialize() {
        if (m_IntakeAndShooter.isIntakePhotogateTriggered() && !Constants.IntakeAndShooter.kIntaked) {
        m_IntakeAndShooter.setIntakeSpeed(1);
        } if(m_IntakeAndShooter.isShooterPhotogateTriggered() && !Constants.IntakeAndShooter.kIntaked){ 
            m_IntakeAndShooter.setIntakeSpeed(-.075);
            if (m_IntakeAndShooter.isIntakePhotogateTriggered() && !m_IntakeAndShooter.isShooterPhotogateTriggered() && !Constants.IntakeAndShooter.kIntaked) {
                m_IntakeAndShooter.setIntakeSpeed(0);
            }
        } 
    }

    @Override
    public void execute() {
        if(m_IntakeAndShooter.isIntakePhotogateTriggered()  && !Constants.IntakeAndShooter.kIntaked){ 
            m_IntakeAndShooter.setShooterSpeed(1);
            Constants.IntakeAndShooter.kIntaked = true;
            if(m_IntakeAndShooter.shooterReady() && Constants.IntakeAndShooter.kIntaked){
                m_IntakeAndShooter.setIntakeSpeed(1);
            }
        }
    }

    @Override
    public boolean isFinished() {
        new WaitCommand(1);
        if (!m_IntakeAndShooter.isIntakePhotogateTriggered() && !m_IntakeAndShooter.isShooterPhotogateTriggered()) {
            m_IntakeAndShooter.setIntakeSpeed(0);
            m_IntakeAndShooter.setShooterSpeed(0);
            Constants.IntakeAndShooter.kIntaked = false;
            return true;
        }
        return false;
    }
}
