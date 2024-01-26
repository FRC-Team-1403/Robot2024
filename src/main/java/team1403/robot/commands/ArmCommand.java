package team1403.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants.Arm;
import team1403.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
   private Arm m_arm;
   private final PIDController m_pivotPid;
   private final DutyCycleEncoder m_armAbsoluteEncoder;
   private double m_armSpeed;
   private double m_pivotAngle;
   private double m_tolerance;


   public ArmCommand(Arm arm, PIDController pivotPid, DutyCycleEncoder armAbsoluteEncoder, double armSpeed, double pivotAngle,double tolerance ) {
       m_arm = arm;
       m_armSpeed = armSpeed;
       m_pivotAngle = pivotAngle;
       m_pivotPid = pivotPid;
       m_armAbsoluteEncoder = armAbsoluteEncoder;

        m_tolerance = tolerance;
   }

   @Override
   public void execute() {  
        m_arm.periodic(m_pivotAngle);
   }

   @Override
   public boolean isFinished() {
    
       if (m_tolerance >= Math.abs(m_armAbsoluteEncoder.get() - m_pivotAngle)){
        m_arm.stop();
       return true;}
       return false;
    }


}
