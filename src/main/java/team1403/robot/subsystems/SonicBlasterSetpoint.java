package team1403.robot.subsystems;

public class SonicBlasterSetpoint {

    private double m_armAngle;
    private double m_wristAngle;
    private double m_intakeSpeed;
    private double m_shooterRPM;

    public SonicBlasterSetpoint(double armAngle, double wristAngle, double intakeSpeed, double shooterRPM) {
        m_armAngle = armAngle;
        m_wristAngle = wristAngle;
        m_shooterRPM = shooterRPM;
        m_intakeSpeed = intakeSpeed;
    }

    public double getArmAngle() {
        return m_armAngle;
    }

    public double getWristAngle() {
        return m_wristAngle;
    }

    public double getIntakeSpeed() {
        return m_intakeSpeed;
    }

    public double getShooterRPM() {
        return m_shooterRPM;
    }
    
}
