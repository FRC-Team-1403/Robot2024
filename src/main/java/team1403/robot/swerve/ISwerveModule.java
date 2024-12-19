package team1403.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModule {

    public class SwerveModuleTelemetery {
        public double driveVolt; //volts
        public double driveCurrent; //amps
        public double turnVolt; //volts
        public double turnCurrent; //amps
        public double drivePos; //m
        public double turnPos; //rad
        public double driveVel; //m/s
        public double turnVel; //rad/s
    }

    public String getName();

    public SwerveModuleState getState();

    public SwerveModulePosition getModulePosition();

    public void set(double driveMetersPerSecond, double steerAngle);

    //SysID methods, which is optional so we have some do nothing methods
    //can be moved to a seperate interface later
    public default void setDriveVoltage(double volts) { }
    //only disabled closed loop on drive motors. Once again, intended for sysid, but can use for debugging too :)
    public default void disableClosedLoop(boolean disable) {}
    public default SwerveModuleTelemetery getData(SwerveModuleTelemetery t) { return t; }
}
