package team1403.lib.util;


/**
 * An interface for getting time.
 */
public interface Clock {
  /**
   * Microseconds to milliseconds.
   */
  public static final double kMillisPerMicro = 0.001;

  /**
   * Milliseconds to seconds.
   */
  public static final double kSecsPerMilli = 0.001;

  /**
   * Microseconds to seconds.
   */
  public static final double kSecsPerMicro = kSecsPerMilli * kMillisPerMicro;

  /**
   * Seconds to milliseconds.
   */
  public static final long kMillisPerSec = 1000;

  /**
   *  Milliseconds to microseconds.
   */
  public static final long kMicrosPerMilli = 1000;

  /**
   * Seconds to microseconds.
   */
  public static final long kMicrosPerSec = kMicrosPerMilli * kMillisPerSec;

  /**
   * Get the current time.
   *
   * <p>Note that the time is relative to some undefined base, but will
   * be consistent over the lifetime of the running JVM. It could be
   * when the JVM started. Therefore this time should not be interpreted
   * other than in comparison with other times returned from this same
   * clock instance.
   *
   * @return seconds
   */
  double nowSecs();

  /**
   * Current time in milliseconds.
   *
   * @return milliseconds
   */
  double nowMillis();

  /**
   * Current time in microseconds.
   *
   * @return microseconds
   */
  long nowMicros();
}
