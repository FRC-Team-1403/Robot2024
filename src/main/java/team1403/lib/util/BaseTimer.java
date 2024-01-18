
package team1403.lib.util;

/**
 * Implements a Timer.
 *
 * <p>This class adds some convienience functions to the base interface
 * for retrieveing and recording time. The units used for recording time
 * are independent for the units requested when retreiving the time. This
 * way we can use human-readable measurements but still use microsecond
 * measurements internally if they are more appropriate for the use case.
 *
 * <p>Sample usage:
 * {@code
 *    Clock clock = WpiClock.instance();
 *    BaseTimer timer = new BaseTimer("MyTimer", clock);
 *    ...
 *    long micros = timer.timer.nowMicros();  // only visible in code.
 *    ...
 *    long finalMicros = timer.recordMicros();  // visible to dashboard.
 * }
 */
public class BaseTimer implements Timer {
  /**
   * RecordUnits designate the units to use when recording measurements.
   */
  public enum RecordUnit {
    /**
     * Denotes that time durations should be converted into milliseconds.
     */
    MILLISECOND,

    /**
     * Denotes that time durations should be converted into microseconds.
     */
    MICROSECOND,
  }

  /**
   * Constructor.
   *
   * <p>Records in MILLISECOND units
   *
   * @param name The metric name to use when recording measurements.
   * @param clock The clock provides the time we are measuring with.
   */
  public BaseTimer(String name, Clock clock) {
    this(name, clock, RecordUnit.MILLISECOND);
  }

  /**
   * Constructor.
   *
   * @param name The metric name to use when recording measurements.
   * @param clock The clock provides the time we are measuring with.
   * @param unit The units to use when recording measurements.
   */
  public BaseTimer(String name, Clock clock, RecordUnit unit) {
    m_name = name;
    m_clock = clock;
    m_startMicros = m_clock.nowMicros();
    m_unit = unit;
  }

  /**
   * Returns the metric name used when recording.
   *
   * @return name bound by constructor.
   */
  public final String getName() {
    return m_name;
  }

  /**
   * Return the elapsed time in microseconds and record the measurement.
   *
   * <p>Records the measurement using the unit specified at construction.
   *
   * @return microseconds regardless of recording unit.
   */
  public final long recordMicros() {
    long micros = nowMicros();
    if (m_unit == RecordUnit.MILLISECOND) {
      recordAsMillis(m_name, micros * Clock.kMillisPerMicro);
    } else {
      recordAsMicros(m_name, micros);
    }
    return micros;
  }

  /**
   * Return the elapsed time in milliseconds and record the measurement.
   *
   * <p>Records the measurement using the units specified at construction.
   *
   * @return milliseconds regardless of recording unit.
   */
  public final double recordMillis() {
    return recordMicros() * Clock.kMillisPerMicro;
  }

  /**
   * Time how long it takes to run a method in microseconds.
   *
   * <p>This does not depend on nor affect ellapsed time.
   *
   * @param record Whether to record the measurement.
   *               The measurement will be recorded in the units
   *               specified by the constructor.
   * @param runnable Provides the method to run.
   *
   * @return microseconds regardless of if/how it is recorded.
   */
  public long timeCallMicros(boolean record, Runnable runnable) {
    long start = m_clock.nowMicros();
    runnable.run();
    long micros = m_clock.nowMicros() - start;
    if (record) {
      if (m_unit == RecordUnit.MILLISECOND) {
        recordAsMillis(m_name, micros * Clock.kMillisPerMicro);
      } else {
        recordAsMicros(m_name, micros);
      }
    }
    return micros;
  }

  /**
   * Time how long it takes to run a method in milliseconds.
   *
   * <p>This does not depend on nor affect ellapsed time.
   *
   * @param record Whether to record the measurement.
   *               The measurement will be recorded in the units
   *               specified by the constructor.
   * @param runnable Provides the method to run.
   *
   * @return milliseconds regardless of if/how it is recorded.
   */
  public double timeCallMillis(boolean record, Runnable runnable) {
    return timeCallMicros(record, runnable) * Clock.kMillisPerMicro;
  }

  @Override
  public void restart() {
    m_startMicros = m_clock.nowMicros();
  }

  @Override
  public long nowMicros() {
    return m_clock.nowMicros() - m_startMicros;
  }

  private final String m_name;
  private final Clock m_clock;
  private final RecordUnit m_unit;
  private long m_startMicros;
}
