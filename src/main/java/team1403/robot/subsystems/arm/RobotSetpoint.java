package team1403.robot.subsystems.arm;

import team1403.robot.subsystems.Blackbox;

public class RobotSetpoint {
    private double m_armAngle;
    private double m_wristAngle;
    private double m_shooterRPM;

    public RobotSetpoint(double armAngle, double wristAngle, double shooterRPM) {
        set(armAngle, wristAngle, shooterRPM);
    }

    public void set(double armAngle, double wristAngle, double shooterRPM) { 
        m_armAngle = armAngle;
        m_wristAngle = wristAngle;
        m_shooterRPM = shooterRPM;
    }

    public void applySetpoint() {
        Blackbox.setShooterRPM(m_shooterRPM);
        Blackbox.setArmAngle(m_armAngle);
        Blackbox.setWristAngle(m_wristAngle);
    }
}
