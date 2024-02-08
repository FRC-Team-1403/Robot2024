package team1403.robot.commands;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Intake;


public class RunIntake extends Command {
   private Intake m_intake;
   private double m_intakeSpeed;
   private DigitalInput m_intakePhotogate1;
   private DigitalInput m_Photogate2;
   

   public RunIntake(Intake intake, double intakeSpeed, DigitalInput intakePhotogate, DigitalInput Photogate2) {
       m_intake = intake;
       m_intakeSpeed = intakeSpeed;
       m_intakePhotogate1 = intakePhotogate;
       m_Photogate2 = Photogate2;
   }

   @Override
   public void initialize() {
       m_intake.setIntakeSpeed(m_intakeSpeed);
   }

   //slows down intake motor when intakePhotogate1 is hit 
   @Override
   public void execute() {  
        if(m_intakePhotogate1.get()){
            m_intake.setIntakeSpeed(m_intakeSpeed / Constants.Intake.kDecreaseIntakeSpeed); 
        } 
   }
   @Override
   public boolean isFinished() {
       if (m_Photogate2.get()) {
         m_intake.stop();
       }
       return m_Photogate2.get();
    }

}
