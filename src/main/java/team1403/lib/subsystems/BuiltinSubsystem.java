package team1403.lib.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import team1403.lib.core.CougarLibInjectedParameters;
import team1403.lib.core.CougarSubsystem;
import team1403.lib.device.CougarAccelerometer;
import team1403.lib.device.PowerDistributor;

/**
 * The BuiltinSubsystem contains ubiquitous builtin devices.
 *
 * <p>These are not necessarily free so this subsystem does
 * not automatically report metrics or all details. You must
 * ask for the things you want.
 */
public class BuiltinSubsystem extends CougarSubsystem {
  /**
   * Constructor.
   *
   * @param injectedParameters The injected CougarLib parameters.
   */
  public BuiltinSubsystem(CougarLibInjectedParameters injectedParameters) {
    super("BuiltinDevices", injectedParameters);

    var factory = injectedParameters.getDeviceFactory();
    m_pdp = factory.makePowerDistributor("BuiltinDevices.Pdp");
    m_accelerometer = factory.makeBuiltinAccelerometer("BuiltinDevices.Accelerometer",
                                                       CougarAccelerometer.Range.k4G);

    addDevice("PDP", m_pdp);
    addDevice("Accelerometer", m_accelerometer);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("BuiltinDevices.Pdp.Temp", m_pdp.getCelsius());
    SmartDashboard.putNumber("BuiltinDevices.Pdp.Amps", m_pdp.getAmps());
    SmartDashboard.putNumber("BuiltinDevices.Pdp.Volts", m_pdp.getVoltage());
    SmartDashboard.putNumber("BuiltinDevices.Pdp.Joules", m_pdp.getTotalJoules());

    // Let's show memory usage (in MB) for curiosity.
    // There doesnt seem to be a wawy to see in-use so we will show free.
    var runtime = Runtime.getRuntime();
    SmartDashboard.putNumber("Java.FreeMB",
                             ((double)runtime.freeMemory()) / (1000.0 * 1000.0));
  }

  /**
   * Returns builtin accelerometer.
   *
   * @return builtin Accelerometer.
   */
  public CougarAccelerometer getAccelerometer() {
    return m_accelerometer;
  }

  /**
   * Returns builtin power distributor.
   *
   * @return builtin PowerDistributor
   */
  public PowerDistributor getPowerDistributor() {
    return m_pdp;
  }

  private final PowerDistributor m_pdp;
  private final CougarAccelerometer m_accelerometer;
}
