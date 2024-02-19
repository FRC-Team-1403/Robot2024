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
    private CANSparkMax m_intake;
    private PIDController m_controller;
    private PIDController m_controller2;
    private DigitalInput m_shooterPhotoGate;
    private DigitalInput m_intakePhotoGate;
    private       DigitalInput          m_test ;

    private double rpm1;
    private double rpm2;
    
    public IntakeSubsystem() {
        m_topShooterMotor = new TalonFX(Constants.Intake.kIntakeNeoTopCanID);
        m_bottomShooterMotor = new TalonFX(Constants.Intake.kIntakeNeoBottomCANID);
        m_intake = new CANSparkMax(Constants.Intake.kIntakeCANID, CANSparkMax.MotorType.kBrushless);
        m_controller = new PIDController(0.00132, .000015, 0);
        m_controller2 = new PIDController(0.00132, 0.00015, 0);
        rpm1 = 0;
                rpm2 = 0;
        m_intake.setIdleMode(IdleMode.kBrake);
        m_bottomShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        m_shooterPhotoGate = new DigitalInput(Constants.Intake.kShooterPhotoGateID);
        m_intakePhotoGate = new DigitalInput(Constants.Intake.kIntakePhotoGateID);
                m_test = new DigitalInput(3);

    }
    public boolean isShooterGateOn() {
        return m_shooterPhotoGate.get();
    }

    public boolean isIntakeGateOn() {
        return m_intakePhotoGate.get();
    }

    public void setShooterRpm(double rpm)
    {
        rpm1 = rpm;
    }

    public double getShooterRpmTop() {
        return  -(m_topShooterMotor.getVelocity().getValueAsDouble() *600/2048) * 60.0;
    }
    public double getShooterRpmBottom() {
        return  -(m_bottomShooterMotor.getVelocity().getValueAsDouble() *600/2048) * 60.0;
    }
    public void setIntakeRpm(double rpm)
    {
        rpm2 = rpm;
    }


    public void setTopNeoSpeed(double speed) {
        m_topShooterMotor.set(speed);
    }

    public void getTopNeoSpeed() {
        m_topShooterMotor.get();
    }

    public void setBottomNeoSpeed(double speed) {
        m_bottomShooterMotor.set(speed);
    }

    public void getBottomNeoSpeed() {
        m_bottomShooterMotor.get();
    }

    public void setTalonSpeed(double speed) {
        m_intake.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        m_intake.set(speed);
    }
    public void setShooterSpeed(double speed) {
        m_bottomShooterMotor.set(speed);
        m_topShooterMotor.set(speed);
    }

    public void setEverythingSpeed(double speed) {
        setIntakeSpeed(speed);
        setShooterSpeed(speed);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("PhotoGate1", m_intakePhotoGate.get());
                SmartDashboard.putBoolean("PhotoGate2", m_shooterPhotoGate.get());
        SmartDashboard.putBoolean("Test", m_test.get());
        double deltaSpeed2 = m_controller2.calculate(getShooterRpmTop(), rpm1);

        double deltaSpeed = m_controller2.calculate(getShooterRpmBottom(), rpm1);
        m_bottomShooterMotor.set(-deltaSpeed);
        m_topShooterMotor.set(-deltaSpeed2);
        SmartDashboard.putNumber("intake rpm", rpm2);   
        SmartDashboard.putNumber("shooter rpm", rpm1);

        SmartDashboard.putNumber("intake speed", m_intake.get()); 
        SmartDashboard.putNumber("shooter speed", m_topShooterMotor.get());
        SmartDashboard.putNumber("velocity",getShooterRpmBottom());
    }
}

