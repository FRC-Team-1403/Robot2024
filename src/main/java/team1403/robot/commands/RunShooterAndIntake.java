package team1403.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team1403.robot.Constants;
import team1403.robot.subsystems.IntakeAndShooter.Intake;
import team1403.robot.subsystems.IntakeAndShooter.Shooter;

/**
 * Created the intake and shooter command class.
 */
public class RunShooterAndIntake extends Command {
    private Shooter m_shooter;
    private Intake m_intake;
    private double m_intakeSpeed;
    private DigitalInput m_shooterPhotogate;
    private DigitalInput m_intakePhotogate;

    /**
     *
     * @param shooter the shooter.
     * @param shooterSpeed the speed of the shooter.
     * @param intake the intake.
     * @param intakeSpeed the speed of the intake.
     * @param shooterPhotogate the photogate closest to the shooter.
     */
    public RunShooterAndIntake(Shooter shooter, double shooterSpeed, Intake intake, double intakeSpeed, DigitalInput shooterPhotogate) {
        m_shooter = shooter;
        m_intake = intake;
        m_intakeSpeed = intakeSpeed;
        m_shooterPhotogate = shooterPhotogate;
    }

    /**
     * setting the intake speed to m_intakeSpeed.
     */
    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(m_intakeSpeed);
    }

    /**
     * what to do when the intake photogate is triggered.
     */
    @Override
    public void execute() {
        if(m_intakePhotogate.get()){
            m_intake.setIntakeSpeed(m_intakeSpeed / Constants.Arm.kDecreaseIntakeSpeed); 
        } 
    }

    /**
     * what to do when the the shooter photogate is triggered.
     */
    @Override
    public boolean isFinished() {
        if (m_shooterPhotogate.get()) {
            new SequentialCommandGroup(
                new WaitCommand(3),
                new InstantCommand(() -> m_intake.intakeStop()),
                new InstantCommand(() -> m_shooter.shooterStop()));
        }
        return m_shooterPhotogate.get();
       }
    }

