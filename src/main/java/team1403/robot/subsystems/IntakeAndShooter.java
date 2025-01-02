package team1403.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< Updated upstream
import team1403.lib.device.wpi.CougarSparkMax;
<<<<<<< Updated upstream
=======
>>>>>>> Stashed changes
import team1403.lib.util.CougarLogged;



/** creating the intake and shooter class */

public class IntakeAndShooter extends SubsystemBase implements CougarLogged {  

  //Motors & Photoswitches
  private CANSparkMax m_IntakeMotor;
  private TalonFX m_shooterMotorTop;
  private TalonFX m_shooterMotorBottom;
  private DigitalInput m_intakePhotoswitch;
  private DigitalInput m_shooterPhotoswitch;
=======
import team1403.lib.device.wpi.CougarTalonFx;
import team1403.robot.Constants; 

/**
 * creating the intake and shooter class.
 */
public class IntakeAndShooter extends SubsystemBase {  
  // Intake motor
  private static CougarSparkMax m_intakeMotor;
  
  // shooter motors
  private CougarTalonFx m_shooterMotorTop;
  private CougarTalonFx m_shooterMotorBottom;

  // photogates
  private DigitalInput m_intakePhotogate;
  private DigitalInput m_shooterPhotogate;
  private PIDController m_bottomShooter;
  private PIDController m_topShooter;
>>>>>>> Stashed changes


  public IntakeAndShooter() {
    
    m_shooterMotorTop.getEmbeddedEncoder().setVelocityConversionFactor(60.);
    m_shooterMotorBottom.getEmbeddedEncoder().setVelocityConversionFactor(60.);
  }

  /** methods needed for subsystem */
  //start intake motor
  public void startIntakeMotor(double speed){
    m_IntakeMotor.set(speed);
  }

  //stop intake motor
  public void stopIntakeMotor(){
    m_IntakeMotor.set(0);
  }

  //start shooter motor
  public void startShooterMotor(double speed){
    m_shooterMotorTop.set(-speed);
    m_shooterMotorBottom.set(-speed);
  }

  //stop shooter motor
  public void stopShooterMotor(){
    m_shooterMotorTop.set(0);
    m_shooterMotorBottom.set(0);
  }

  //set the rpm
  public void setShooterRPM(double rpm){
    m_shooterMotorTop.set(rpm);
    m_shooterMotorBottom.set(rpm);
  }
  //checking to make sure it is 6000 rpm
  public double getShooterRPMBottom(){
    //only using one of the shooter motors because they both should be the same in an ideal situation
    return m_shooterMotorBottom.get() * 600/2048;
  }

  public double getShooterRPMTop(){
    //only using one of the shooter motors because they both should be the same in an ideal situation
    return m_shooterMotorTop.getSetpoint();
  }

  //checking if the shooter RPM is enough
  public boolean isReady() {
    if (getShooterRPMTop() > 5900 && getShooterRPMBottom() > 5900) {
      return true;
    }
    else{
      return false;
    }
  }
  
  //get the intake photoswitch
  public boolean getIntakePhotoswitchTriggered(){
    return getIntakePhotoswitchTriggered();
  }

  //intake photoswitch boolean
  public boolean intakePhotoswitchTriggered() {
    if (getIntakePhotoswitchTriggered() == true){
      return true;
    } 
    else{
      return false;
    }
  } 

  //get the shooter photoswitch
  public boolean getShooterPhotoswitchTriggered(){
    return getShooterPhotoswitchTriggered();
  }

  //shooter photoswitch boolean
  public boolean shooterPhotoswitchTriggered() {
    if (getShooterPhotoswitchTriggered() == true){
      return true;
    } 
    else{
      return false;
    }
  }

  public void periodic() {
    
  }
}
