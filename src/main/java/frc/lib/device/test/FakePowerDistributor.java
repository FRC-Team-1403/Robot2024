package frc.lib.device.test;

import java.util.HashMap;
import java.util.Map;

import frc.lib.device.BaseDevice;
import frc.lib.device.CurrentSensor;
import frc.lib.device.PowerDistributor;


/**
 * Implements a fake current sensor.
 *
 * <p>You must populate the individual circuit currentSensors with their
 * expected names using {@link #putCurrentSensor}.
 */
public class FakePowerDistributor
       extends BaseDevice
       implements PowerDistributor {
  /**
   * The default number of circuits is the max possible for the interface.
   */
  public static final int kDefaultNumCircuits = 24;

  /**
   * Constructor.
   *
   * @param name The name of the instance.
   * @param numCircuits How many circuits the distribtor supports.
   */
  public FakePowerDistributor(String name, int numCircuits) {
    super(name);
    m_numCircuits = numCircuits;
  }

  /**
   * Add energy consumption.
   *
   * @param joules joules consumed.
   */
  public void addJoules(double joules) {
    m_joules += joules;
  }

  /**
   * Set the amps to be returned.
   *
   * @param amps Amps to return
   */
  public void setAmps(double amps) {
    m_amps = amps;
  }

  /**
   * Set the voltage to be returned.
   *
   * @param volts Voltage to return
   */
  public void setVoltage(double volts) {
    m_volts = volts;
  }

  /**
   * Set the watts to be returned.
   *
   * @param watts Watts to return
   */
  public void setWatts(double watts) {
    m_watts = watts;
  }

  /**
   * Set the temperature to be returned.
   *
   * @param celsius Temperature to return in degrees C
   */
  public void setCelsius(double celsius) {
    m_celsius = celsius;
  }

  /**
   * Set the current sensor to return for the given circuit.
   *
   * @param circuit The circuit identifier.
   * @param sensor The sensor on the circuit.
   */
  public void putCurrentSensor(int circuit, CurrentSensor sensor) {
    m_currentSensors.put(circuit, sensor);
  }

  @Override
  public CurrentSensor makeCurrentSensor(String name, int circuit) {
    if (m_currentSensors.containsKey(circuit)) {
      var sensor = m_currentSensors.remove(circuit);
      if (sensor.getName() != name) {
        throw new IllegalArgumentException(
            "currentSensor name mismatch for circuit " + circuit
            + ".  '" + sensor.getName() + "' != '" + name + "'");
      }
      return sensor;
    }
    throw new IllegalArgumentException("No CircuitSensor provided for " + circuit);
  }

  @Override
  public int countCircuits() {
    return m_numCircuits;
  }

  @Override
  public final double getAmps() {
    return m_amps;
  }

  @Override
  public final double getVoltage() {
    return m_volts;
  }

  @Override
  public final double getWatts() {
    return m_watts;
  }

  @Override
  public final double getCelsius() {
    return m_celsius;
  }

  @Override
  public double getTotalJoules() {
    return m_joules;
  }

  @Override
  public void resetTotalJoules() {
    m_joules = 0;
  }

  private double m_amps = Double.NaN;
  private double m_volts = Double.NaN;
  private double m_watts = Double.NaN;
  private double m_celsius = Double.NaN;
  private double m_joules = 0;
  private final int m_numCircuits;
  private final Map<Integer, CurrentSensor> m_currentSensors = new HashMap<>();  // NOPMD
}
