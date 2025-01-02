package team1403.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.device.wpi.CougarSparkMax;
<<<<<<< Updated upstream
import team1403.lib.util.CougarLogged;


/** creating the intake and shooter class */

public class IntakeAndShooter extends SubsystemBase implements CougarLogged {  

  //Motors & Photoswitches
  private CougarSparkMax m_topIntakeMotor;
  private CougarSparkMax m_bottomIntakeMotor;
  private CougarSparkMax m_shooterMotorTop;
  private CougarSparkMax m_shooterMotorBottom;
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
    m_topIntakeMotor.setSpeed(speed);
    m_bottomIntakeMotor.setSpeed(speed);
  }

  //stop intake motor
  public void stopIntakeMotor(){
    m_topIntakeMotor.setSpeed(0);
    m_bottomIntakeMotor.setSpeed(0);
  }

  //start shooter motor
  public void startShooterMotor(double speed){
    m_shooterMotorTop.setSpeed(-speed);
    m_shooterMotorBottom.setSpeed(-speed);
  }

  //stop shooter motor
  public void stopShooterMotor(){
    m_shooterMotorTop.setSpeed(0);
    m_shooterMotorBottom.setSpeed(0);
  }

  //set the rpm
  public void setShooterRPM(double rpm){
    m_shooterMotorTop.setVelocity(rpm);
    m_shooterMotorBottom.setVelocity(rpm);
  }
  //checking to make sure it is 6000 rpm
  public double getShooterRPM(){
    //only using one of the shooter motors because they both should be the same in an ideal situation
    return m_shooterMotorBottom.setVelocity();
  }
  //intake photoswitch boolean
  public boolean intakePhotoswitchTriggered(){
    return 
  }

  //shooter photoswitch boolean
  public boolean shooterPhotoswitchTriggered(){
    return 
  }

  public void periodic() {
    
  }
}
