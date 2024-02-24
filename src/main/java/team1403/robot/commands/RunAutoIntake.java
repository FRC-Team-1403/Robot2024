package team1403.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.Constants.IntakeAndShooter;

public class RunAutoIntake extends Command{
    private team1403.robot.subsystems.IntakeAndShooter m_intake;
    private double m_intakeSpeed;
    private double m_shooterSpeed;
    private double intakeTime;
    private double shooterTime;
    private double prepTime;
    private Timer m_Timer;
    private boolean finished;
    private boolean m_intaked;

    public RunAutoIntake(team1403.robot.subsystems.IntakeAndShooter intake,double intakeSpeed, double shooterSpeed,boolean intaked){
        m_intake = intake;
        m_intakeSpeed = intakeSpeed;
        m_shooterSpeed =shooterSpeed;
        m_intaked = intaked;
    }

    @Override
    public void initialize() {
        m_Timer.start();
        if(!m_Timer.hasElapsed(intakeTime))m_intake.setIntakeSpeed(0.75);
        m_Timer.reset();
        if(!m_Timer.hasElapsed(prepTime))m_intake.setIntakeSpeed(0.075);
        m_Timer.reset();
        if(!m_Timer.hasElapsed(shooterTime))m_intake.setShooterSpeed(0.5);

   }

   //slows down intake motor when intakePhotogate1 is hit 
   @Override
   public void execute() {  
   }
   @Override
   public boolean isFinished() {
       return finished;
    }
}
