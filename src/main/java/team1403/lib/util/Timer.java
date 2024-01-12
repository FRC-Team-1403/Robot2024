package team1403.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A timer measures how much time has passed since it was restarted.
 *
 * <p>Timers should always start when they are constructed, but be
 * restarted to mark the beginning of the desired interval. This way
 * they are always well defined as running.
 *
 * <p>Timers measure time in microseconds but provide convienence functions
 * to retrieve time in milliseconds. Note that microseconds are `long` where
 * milliseconds are `double` because they still have microsecond resolution.
 * As a rule of thumb, microseconds should be long and milliseconds should be
 * double to avoid mixing up the units. Using microseconds
 *
 * <p>Timers provide the means to record their time as a metric measurement.
 * The default implementation will write these to the SmartDashboard, which
 * is backed by NetworkTables so will show up in DataLogs as well. Typically
 * we will record these values as milliseconds so they are more human readable.
 */
public interface Timer {
  /**
   * Restart the timer so that nowMicros is 0.
   */
  void restart();

  /**
   * Returns microseconds elapsed since restarted (or constructed).
   *
   * @return microseconds
   */
  long nowMicros();

  /**
   * Returns milliseconds elapsed since restarted (or constructed).
   *
   * <p>This method is conceptually final. It is implemented in terms
   * of {@link #nowMicros} so overriding nowMicros is sufficient.
   *
   * @return milliseconds
   */
  default double nowMillis() {
    return nowMicros() * Clock.kMillisPerMicro;
  }

  /**
   * Records a microsecond metric measurement.
   *
   * @param name The name of the metric.
   * @param micros The microsecond value to record.
   */
  default void recordAsMicros(String name, long micros) {
    SmartDashboard.putNumber(name, Long.valueOf(micros));
  }

  /**
   * Records a millisecond metric measurement.
   *
   * @param name The name of the metric.
   * @param millis The millisecond value to record.
   */
  default void recordAsMillis(String name, double millis) {
    SmartDashboard.putNumber(name, Double.valueOf(millis));
  }
}
