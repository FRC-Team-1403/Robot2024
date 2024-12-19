package team1403.robot.swerve;

import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class SimSwerveModule extends SubsystemBase implements ISwerveModule {

    private final String m_name;
    private final SwerveModuleState m_state;
    private final SwerveModulePosition m_position;
    
    private final DCMotorSim m_driveSim;
    private final DCMotorSim m_steerSim;
    private final PIDController m_steerController = new PIDController(6, 0, 0);
    private final PIDController m_driveController = new PIDController(0.04, 0, 0);
    //obtained with SysID
    private final SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(0.011235, 2.5172, 0.3881);
    private double m_driveVolt = 0;
    private double m_turnVolt = 0;

    private static final double PID_PERIOD = 0.004;
    private final Notifier m_pidNotifier;
    private final ReentrantLock m_lock;

    private boolean m_disableDrivePID = false;

    public SimSwerveModule(String name) {
        m_name = name;
        m_position = new SwerveModulePosition();
        m_state = new SwerveModuleState();

        m_driveSim = new DCMotorSim(DCMotor.getNEO(1), 1/Constants.Swerve.kDriveReduction, 0.025);
        m_steerSim = new DCMotorSim(DCMotor.getNEO(1), 1/Constants.Swerve.kSteerReduction, 0.004);

        m_steerController.enableContinuousInput(-Math.PI, Math.PI);

        m_lock = new ReentrantLock();

        m_pidNotifier = new Notifier(this::simLoop);
        m_pidNotifier.setName("SimSwervePID " + name);
        m_pidNotifier.startPeriodic(PID_PERIOD);
    }

    private double getDriveVelocity() {
        return m_driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.kWheelRadiusMeters; //omega * r = v
    }

    private double getDrivePosition() {
        return m_driveSim.getAngularPositionRad() * Constants.Swerve.kWheelRadiusMeters;
    }

    @Override
    public String getName() {
        return m_name;
    }

    @Override
    public SwerveModuleState getState() {
        m_state.angle = Rotation2d.fromRadians(MathUtil.angleModulus(m_steerSim.getAngularPositionRad()));
        m_state.speedMetersPerSecond = getDriveVelocity();

        return m_state;
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        m_position.angle = Rotation2d.fromRadians(MathUtil.angleModulus(m_steerSim.getAngularPositionRad()));
        m_position.distanceMeters = getDrivePosition();

        return m_position;
    }

    @Override
    public void set(double driveMetersPerSecond, double steerAngle) {
        m_lock.lock();
            m_driveController.setSetpoint(driveMetersPerSecond);
            m_steerController.setSetpoint(MathUtil.angleModulus(steerAngle));
        m_lock.unlock();
    }

    public void disableClosedLoop(boolean disable) {
        m_disableDrivePID = disable;
    }

    public void setDriveVoltage(double voltage) {
        m_driveSim.setInputVoltage(voltage);
        m_driveVolt = voltage;
    }

    public SwerveModuleTelemetery getData(SwerveModuleTelemetery t) {
        t.driveCurrent = m_driveSim.getCurrentDrawAmps();
        t.driveVolt = m_driveVolt;
        t.driveVel = getDriveVelocity();
        t.drivePos = getDrivePosition();
        t.turnCurrent = m_steerSim.getCurrentDrawAmps();
        t.turnVolt = m_turnVolt;
        t.turnVel = m_steerSim.getAngularVelocityRadPerSec();
        t.turnPos = m_steerSim.getAngularPositionRad();
  
        return t;
    }

    private void simLoop() {
        m_lock.lock();
        double driveVel = getDriveVelocity(); 
        m_driveVolt = m_driveController.calculate(driveVel) + 
            m_driveFF.calculate(driveVel, m_driveController.getSetpoint(), Constants.kLoopTime);

        if (!m_disableDrivePID) m_driveSim.setInputVoltage(m_driveVolt);

        m_turnVolt = m_steerController.calculate(m_steerSim.getAngularPositionRad());

        m_steerSim.setInputVoltage(m_turnVolt);
        m_lock.unlock();

        m_driveSim.update(PID_PERIOD);
        m_steerSim.update(PID_PERIOD);
    }
    
}
