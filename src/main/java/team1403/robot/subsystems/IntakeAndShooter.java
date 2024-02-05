package team1403.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.device.AdvancedMotorController;
import team1403.lib.device.wpi.CougarSparkMax;
import team1403.lib.device.wpi.CougarTalonFx;
import team1403.robot.Constants;

/**
 * creating the intake and shooter class.
 */
public class IntakeAndShooter extends SubsystemBase {

  //constant that should be in constants >:(
  private double lastSpeed = 0;

  /**
   * intake class.
   */
  public class Intake {

    //Intake motor
    private static CougarSparkMax m_intakeMotor;

    //photogate
    private DigitalInput m_intakePhotogate;

    /**
     * creating motor and sensors for the intake.
     *
     * @param injectedParameters injected parameters.
     */
    public Intake(CougarLibInjectedParameters injectedParameters) {
    
    //intake motors and sensors
    m_intakeMotor = CougarSparkMax.makeBrushless("Top Intake Motor", Constants.CanBus.intakeMotorTop, SparkRelativeEncoder.Type.kHallSensor);
    m_intakePhotogate = new DigitalInput(Constants.RioPorts.intakePhotogate1);

    /**
     * un-needed stuff
     */
    //m_intakeMotorBottom = CougarSparkMax.makeBrushless("Bottom Intake Motor", Constants.CanBus.intakeMotorBottom, SparkRelativeEncoder.Type.kHallSensor);
    //m_intakeLimitSwitch = new WpiLimitSwitch("Intake Limit Switch", Constants.RioPorts.intakeLimitSwitchPort);
    //m_intakePhotogate2 = new DigitalInput(Constants.RioPorts.kIntakePhotogae2);
    }

    /**
     * state of the intake.
     *
     * @return true or false depending on if it is trigered.
     */
    public boolean isIntakePhotogateTriggered() {
      return m_intakePhotogate.get();
    }

    /**
     * if the intake is ready.
     *
     * @return true or false
     */
    public boolean intakeReady() {
      if (lastSpeed == Math.abs(m_intakeMotor.getEncoder().getVelocity()) && lastSpeed == Math.abs(m_intakeMotor.getEncoder().getVelocity())) {
        return true;
      }
      else {
        return false;
      }  
    }
    
    /**
     * I don't think we need this.
     *
     * @return
     */
    // public boolean intakeSpeedIsEqual() {
    //   if (Math.abs(m_intakeMotor.getEncoder().getVelocity()) == Math.abs(m_intakeMotor.getEncoder().getVelocity())) {
    //       return true;
    //   } else {
    //       return false;
    //   }
    // }

    /**
     * stopping intake.
     */
    public void intakeStop() {
      m_intakeMotor.setSpeed(0);
    }

    /**
     * setting intake speed
     *
     * @param speed
     */
    public void setIntakeSpeed(double speed) {
      //if (m_intakeLimitSwitch.get()) {
        //stop();
        //return;
      //}
      lastSpeed = speed;
      m_intakeMotor.set(speed);
      //if there is an error when testing (note doesn't get taken in) try changing the direction of the motor
    
    }
  }


  /**
   * creating shooter class.
   */
  public class Shooter {

    //shooter photogate
    private DigitalInput m_shooterPhotogate;

    //shooter motors
    private CougarTalonFx m_shooterMotorTop;
    private CougarTalonFx m_shooterMotorBottom;

    /**
     * creating shooter devices.
     *
     * @param injectedParameters
     */
    public Shooter(CougarLibInjectedParameters injectedParameters) {
    //un-needed stuff
    //m_intakeMotorBottom = CougarSparkMax.makeBrushless("Bottom Intake Motor", Constants.CanBus.intakeMotorBottom, SparkRelativeEncoder.Type.kHallSensor);
    //m_intakeLimitSwitch = new WpiLimitSwitch("Intake Limit Switch", Constants.RioPorts.intakeLimitSwitchPort);
    //m_intakePhotogate2 = new DigitalInput(Constants.RioPorts.kIntakePhotogae2);
   
    //shooter motors and sensors
    m_shooterMotorTop = new CougarTalonFx("Top Shooter Motor", Constants.CanBus.shooterMotorBottom);
    m_shooterMotorBottom = new CougarTalonFx("Bottom Shooter Motor", Constants.CanBus.shooterMotorBottom);
    m_shooterPhotogate = new DigitalInput(Constants.RioPorts.shooterPhotogate);
    }

    /**
     * is shooter photogate triggered.
     *
     * @return state of the shooter photogate.
     */
    public boolean isShooterPhotogateTriggered() {
      return m_shooterPhotogate.get();
    }
  
    /**
     * if the shooter is ready.
     */
    public boolean shooterReady() {
      if (lastSpeed == Math.abs(m_shooterMotorTop.getEmbeddedEncoder().getVelocityValue()) 
          && lastSpeed == Math.abs(m_shooterMotorBottom.getEmbeddedEncoder().getVelocityValue())) {
        return true;
      }
      else {
        return false;
      }  
    }

    /**
     * Checking if the speeds of the top ond bottom shooter motor are the same.
     *
     * @return true or false.
     */
    public boolean shooterSpeedIsEqual() {
      if (Math.abs(m_shooterMotorTop.getEmbeddedEncoder().getVelocityValue()) == Math.abs(m_shooterMotorBottom.getEmbeddedEncoder().getVelocityValue())) {
          return true;
      } else {
          return false;
      }
    }

    /**
     * Stopping the shooter motors.
     */
    public void shooterStop() {
      m_shooterMotorTop.setSpeed(0);
      m_shooterMotorBottom.setSpeed(0);
    }

    /**
     * setting the shooter speed.
     * @param speed
     */
    public void setShooterSpeed(double speed) {
      lastSpeed = speed;
      m_shooterMotorTop.setSpeed(-(speed));
      m_shooterMotorBottom.setSpeed(speed);
      //if there is an error when testing (note doesn't get shot out) try changing the direction of the motor
    
    }
  }
  
  public void periodic() {
    Logger.recordOutput("Intake Top Motor Temp", Intake.m_intakeMotor.getMotorTemperature());
    Logger.recordOutput("Intake Top Motor RPM", Intake.m_intakeMotor.getVoltageCompensationNominalVoltage());
  }
}
