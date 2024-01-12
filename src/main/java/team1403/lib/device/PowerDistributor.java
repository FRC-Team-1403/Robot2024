package team1403.lib.device;

/**
 * PowerDistributor
 *
 * <p>Denotes a power distribution panel (or hub).
 *
 * <p>Note that WPI library supports both CTRE and REV devices. The REV device
 * has switchable channels. This capability is not reflected in this interface
 * because we use CTRE PDP so do not need them.
 *
 * <p>If we were to use REV devices, then this interface should be modified
 * (or extended to add switchable channel capabiliity).
 */
public interface PowerDistributor extends CurrentSensor, TemperatureSensor {
  /**
   * Returns the current voltage being drawn by the distributor.
   *
   * @return volts
   */
  double getVoltage();

  /**
   * Returns the number of individual circuits within the distributor.
   *
   * @return number of circuits supports -- they may not all be in use.
   */
  int countCircuits();

  /**
   * Returns a CurrentSensor for an indivual channel within the distributor.
   *
   * @param name Name to assign to the current sensor.
   * @param circuit One of the individual power circuits within the distributor.
   * @return CurrentSensor scoped to that individual circuit.
   */
  CurrentSensor makeCurrentSensor(String name, int circuit);

  /**
   * Returns the total power being consumed.
   *
   * @return watts
   */
  double getWatts();  // power

  /**
   * Return the total energy drawn since last reset.
   *
   * @return joules
   */
  double getTotalJoules();

  /**
   * Reset the accumulator used for getTotalJoules.
   */
  void resetTotalJoules();
}
