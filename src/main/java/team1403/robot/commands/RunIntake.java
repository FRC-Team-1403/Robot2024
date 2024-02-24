package team1403.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeAndShooter;


public class RunIntake extends Command {
   private IntakeAndShooter m_intake;
   private double m_intakeSpeed;

   public RunIntake(IntakeAndShooter intake, double intakeSpeed) {
        this.m_intake = intake;
        m_intakeSpeed = intakeSpeed;
        // m_time = time;
   }

   @Override
   public void initialize() {
       m_intake.setIntakeSpeed(m_intakeSpeed);
   }

   //slows down intake motor when intakePhotogate1 is hit 
   @Override
   public void execute() {  
            m_intake.setIntakeSpeed(m_intakeSpeed / Constants.IntakeAndShooter.kSpeedReduction);      
   }
   @Override
   public boolean isFinished() {
       if (m_intake.isShooterPhotogateTriggered()) {
         m_intake.intakeStop();
       }
       return m_intake.isShooterPhotogateTriggered();
    }

}
