package team1403.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import team1403.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX m_topShooterMotor;
    private TalonFX m_bottomShooterMotor;
    private CANSparkMax m_intake;
    private PIDController m_controller;
    private PIDController m_controller2;
    private DigitalInput m_shooterPhotoGate;
    private DigitalInput m_intakePhotoGate;
    private double intakeRPM;
    private double shooterRPM;

    
    public IntakeSubsystem() {
        m_topShooterMotor = new TalonFX(Constants.Intake.kIntakeNeoTopCanID);
        m_bottomShooterMotor = new TalonFX(Constants.Intake.kIntakeNeoBottomCANID);
        m_intake = new CANSparkMax(Constants.Intake.kIntakeCANID, CANSparkMax.MotorType.kBrushless);
        m_controller = new PIDController(0.00148, .000078, 0);
        m_controller2 = new PIDController(0.00148, 0.00078,0);

        m_topShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        m_bottomShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        m_intake.setIdleMode(IdleMode.kBrake);
        m_shooterPhotoGate = new DigitalInput(Constants.Intake.kShooterPhotoGateID);
        m_intakePhotoGate = new DigitalInput(Constants.Intake.kIntakePhotoGateID);
    }
    public boolean isShooterGateOn() {
        return m_shooterPhotoGate.get();
    }

    public boolean isIntakeGateOn() {
        return m_intakePhotoGate.get();
    }

    public void setShooterRPM(double rpm) {
        shooterRPM = rpm;
    }

    public double getShooterRPMTop() {
        return  -(m_topShooterMotor.getVelocity().getValueAsDouble() *600/2048) * 60.0;
    }

    public double getShooterRPMBottom() {
        return  -(m_bottomShooterMotor.getVelocity().getValueAsDouble() *600/2048) * 60.0;
    }

    public void setIntakeSpeed(double speed) {
        m_intake.set(speed);
    } 
    public void stopAll() {
        setIntakeSpeed(0);
        setShooterSpeed(0);
        setShooterRPM(0);
    }

    public void setShooterSpeed(double speed) {
        m_bottomShooterMotor.set(speed);
        m_topShooterMotor.set(speed);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("PhotoGate1", m_intakePhotoGate.get());
                SmartDashboard.putBoolean("PhotoGate2", m_shooterPhotoGate.get());
        double deltaSpeedTop = m_controller2.calculate(getShooterRPMTop(), shooterRPM);
        double deltaSpeedBottom = m_controller2.calculate(getShooterRPMBottom(), shooterRPM);
        SmartDashboard.putNumber("intake rpm", intakeRPM);   
        SmartDashboard.putNumber("shooter rpm", intakeRPM);
        m_bottomShooterMotor.set(MathUtil.clamp(-deltaSpeedBottom, -1, 1));
        m_topShooterMotor.set(MathUtil.clamp(-deltaSpeedTop, -1, 1));
        SmartDashboard.putNumber("intake speed", m_intake.get()); 
        SmartDashboard.putNumber("shooter speed", m_topShooterMotor.get());
        SmartDashboard.putNumber("velocity", getShooterRPMTop());
    }
}

