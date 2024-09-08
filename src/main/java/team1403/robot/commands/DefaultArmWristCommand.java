package team1403.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.arm.ArmSubsystem;
import team1403.robot.subsystems.arm.Wrist;

public class DefaultArmWristCommand extends Command {

    private ArmSubsystem m_arm;
    private Wrist m_wrist;

    public DefaultArmWristCommand(ArmSubsystem arm, Wrist wrist) {
        m_arm = arm;
        m_wrist = wrist;

        addRequirements(arm, wrist);
    }

    @Override
    public void execute() {
        //TODO: add constants for these numbers, tuning

        double targetArm = Blackbox.getArmAngle();
        double targetWrist = Blackbox.getWristAngle();

        if(m_arm.getPivotAngle() < 107) {
            targetWrist = MathUtil.clamp(targetWrist, 130, 140);
        }
        else if(targetArm < 107 && (m_wrist.getWristAngle() < 130 || m_wrist.getWristAngle() > 140)) {
            targetArm = MathUtil.clamp(targetArm, 107, Constants.Arm.kMaxPivotAngle);
        }

        m_arm.moveArm(targetArm);
        m_wrist.setWristAngle(targetWrist);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
