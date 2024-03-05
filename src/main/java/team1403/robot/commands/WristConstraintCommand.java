package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;  


public class WristConstraintCommand extends Command{
    private Wrist m_wrist;
    private ArmSubsystem m_arm;
    private double m_armSetpoint;
    private double m_wristSetpoint;

    public WristConstraintCommand(Wrist wrist, ArmSubsystem arm,double armSetpoint, double wristSetpoint) {
        this.m_wrist = wrist;
        this.m_arm = arm;
        this.m_armSetpoint = armSetpoint;
        this.m_wristSetpoint = wristSetpoint;
    }

    /*
     * prevents the arm from moving down into tuck position if the wrist is out
     */
    @Override
    public void execute(){

    if(m_arm.getPivotAngle() < Constants.Wrist.kArmConstraint && 
    (m_wrist.getWristAngle() > Constants.Wrist.kWristUpperLimit || m_wrist.getWristAngle() < Constants.Wrist.kWristLowerLimit)){
        m_wrist.setWristAngle(Constants.Wrist.kWristConstraint);
    }
        m_wrist.setWristAngle(m_wristSetpoint);
        m_arm.moveArm(m_armSetpoint);
    }

}





