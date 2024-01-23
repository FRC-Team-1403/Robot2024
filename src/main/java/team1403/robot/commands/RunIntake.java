package team1403.robot.commands;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.robot.subsystems.Intake;


public class RunIntake extends Command {
   private Intake m_intake;
   private double m_intakeSpeed;
   private DigitalInput m_intakePhotogate;



   public RunIntake(Intake intake, double intakeSpeed, DigitalInput intakePhotogate) {
       m_intake = intake;
       m_intakeSpeed = intakeSpeed;
       m_intakePhotogate = intakePhotogate;
   }


   @Override
   public void initialize() {
       m_intake.setIntakeSpeed(m_intakeSpeed);
   }

   @Override
   public void execute() {  
        m_intake.setIntakeSpeed(m_intakeSpeed);
   }

   @Override
   public boolean isFinished() {
       if (m_intakePhotogate.get()) {
         m_intake.stop();
       }
       return m_intakePhotogate.get();
    }


}
