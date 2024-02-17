package team1403.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import team1403.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX m_topShooterMotor;
    private TalonFX m_bottomShooterMotor;
    private CANSparkMax m_intakeMotor;
    private PIDController m_controller;
    private PIDController m_controller2;
    private DigitalInput m_shooterPhotoGate;
    private DigitalInput m_intakePhotoGate;
    private DigitalInput m_test;

    private boolean m_isShooterFinished = false;
    private boolean m_loaded = false;

    private double targetSpeed;
    private double shooterRPM;
    private double intakeRPM;
    
    public IntakeSubsystem() {
        
        m_topShooterMotor = new TalonFX(Constants.Intake.kIntakeNeoTopCanID);
        m_bottomShooterMotor = new TalonFX(Constants.Intake.kIntakeNeoBottomCANID);
        m_intakeMotor = new CANSparkMax(Constants.Intake.kIntakeCANID, CANSparkMax.MotorType.kBrushless);
        m_controller = new PIDController(0.00001, 0, 0);
        m_controller2 = new PIDController(0.00001, 0, 0);
        shooterRPM = 0;
        intakeRPM = 0;
        m_shooterPhotoGate = new DigitalInput(Constants.Intake.kShooterPhotoGateID);
        m_intakePhotoGate = new DigitalInput(Constants.Intake.kIntakePhotoGateID);

    }

    public void intakeStop() {
        m_intakeMotor.set(0.0);
    }

    public void shooterStop() {
        m_topShooterMotor.set(0.0);
        m_bottomShooterMotor.set(0.0);
    }
    
    public boolean isShooterSwitchTripped() {
        return m_shooterPhotoGate.get();
    }

    public boolean isIntakeSwitchTripped() {
        return m_intakePhotoGate.get();
    }

    public void setShooterSpeed(double speed) {
        targetSpeed = speed;
        m_topShooterMotor.set(-(speed));
        m_bottomShooterMotor.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    public boolean isShooterFinished() {
        return m_isShooterFinished;
    }

    public boolean isLoaded() {
        return m_loaded;
    }

    public void everythingStop() {
        intakeStop();
        shooterStop();
    }

    @Override
    public void periodic()
    {
        if (isShooterSwitchTripped()) {
            while (isIntakeSwitchTripped()) {
                setIntakeSpeed(-0.1);
            }
        }

        if (!isIntakeSwitchTripped() && isShooterSwitchTripped()) {
            m_loaded = false;
            m_isShooterFinished = true;
        }
        else if (isIntakeSwitchTripped() && isShooterSwitchTripped()) {
            m_isShooterFinished = false;
        }

        if (isIntakeSwitchTripped()) {
            m_loaded = true;
        }

        SmartDashboard.putBoolean("Intake Photoswitch", isIntakeSwitchTripped());
        SmartDashboard.putBoolean("Shooter Photoswitch", isShooterSwitchTripped());
        SmartDashboard.putBoolean("Test", m_test.get());

        //double deltaSpeed = m_controller.calculate(m_intakeMotor.getEncoder().getVelocity(), intakeRPM);

        //double deltaSpeed2 = m_controller2.calculate(-m_topShooterMotor.getVelocity().getValueAsDouble() * 60, shooterRPM);

        //deltaSpeed2 = m_controller2.calculate(-m_bottomShooterMotor.getVelocity().getValueAsDouble() * 60, shooterRPM);

        SmartDashboard.putNumber("intake rpm", intakeRPM);   
        SmartDashboard.putNumber("shooter rpm", shooterRPM);

        SmartDashboard.putNumber("intake speed", m_intakeMotor.get()); 
        SmartDashboard.putNumber("shooter speed", m_topShooterMotor.get());
    }
}

