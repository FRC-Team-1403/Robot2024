package team1403.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants.Arm;
import team1403.robot.subsystems.arm.ArmSubsystem;

public class ArmCommand extends Command {
   private ArmSubsystem m_arm;
   private double m_pivotAngle;
   private double m_tolerance;


   public ArmCommand(ArmSubsystem arm, double pivotAngle, double tolerance) {
       m_arm = arm;
       m_pivotAngle = pivotAngle;
       m_tolerance = tolerance;
   }

   @Override
   public void initialize() {
        m_arm.moveArm(m_pivotAngle);
   }

   @Override
   public void execute() {
   }
   
   @Override
   public boolean isFinished() {  
        return Math.abs(m_arm.getPivotAngle() - m_pivotAngle) <= m_tolerance;
    }


}
