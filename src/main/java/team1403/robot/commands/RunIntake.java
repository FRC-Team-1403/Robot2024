package team1403.robot.commands;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeAndShooter;


public class RunIntake extends Command {
   private IntakeAndShooter intake_shooter;
   private double m_intakeSpeed;
   private DigitalInput m_intakePhotogate1;
   private DigitalInput m_Photogate2;
   

   public RunIntake(IntakeAndShooter intake_shooter, double intakeSpeed, DigitalInput intakePhotogate, DigitalInput Photogate2) {
       this.intake_shooter = intake_shooter;
       m_intakeSpeed = intakeSpeed;
       m_intakePhotogate1 = intakePhotogate;
       m_Photogate2 = Photogate2;
   }

   @Override
   public void initialize() {
       intake_shooter.setIntakeSpeed(m_intakeSpeed);
   }

   //slows down intake motor when intakePhotogate1 is hit 
   @Override
   public void execute() {  
        if(m_intakePhotogate1.get())
        {
            intake_shooter.setIntakeSpeed(m_intakeSpeed / Constants.IntakeAndShooter.kSpeedReduction);      
        } 
   }
   @Override
   public boolean isFinished() {
       if (m_Photogate2.get()) {
         intake_shooter.intakeStop();
       }
       return m_Photogate2.get();
    }

}
