package team1403.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.util.CougarLogged;
import team1403.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * creating the intake and shooter class.
 */
public class IntakeAndShooter extends SubsystemBase implements CougarLogged {  
  //motor declarations
  public TalonFX m_topShooterWheels;
  public TalonFX m_bottomShooterWheels;
  public CANSparkMax m_intakeWheels;
  public DigitalInput m_photogateShooter; //second photogate
  public DigitalInput m_photogateIntake; //first photogate

  // note rolls back until first is true and second is false

  public IntakeAndShooter() { //method for all of the parameters
  //initializing in parameters
  m_topShooterWheels = new TalonFX(Constants.CanBus.shooterMotorTopID);
  m_bottomShooterWheels = new TalonFX(Constants.CanBus.shooterMotorBottomID);
  m_intakeWheels = new CANSparkMax(Constants.CanBus.intakeMotorID, MotorType.kBrushless);
  m_photogateShooter = new DigitalInput(Constants.RioPorts.shooterPhotogate);
  m_photogateIntake = new DigitalInput(Constants.RioPorts.intakePhotogate1);
}
  
public void setShooterMotor(double speed, TalonFX motor) {
  motor.set(speed);
}

public void setIntakeMotor(double speed, CANSparkMax motor) {
  motor.set(speed);
}

public boolean isPhotoGateTriggered(DigitalInput photogate) {
  return photogate.get();
}


}