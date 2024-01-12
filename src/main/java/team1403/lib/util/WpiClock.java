package team1403.lib.util;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Timer;


/**
 * Implements a Clock using the high resolution FPGA timer.
 *
 * <p>This class is a singleton so provides a static instance.
 */
public final class WpiClock implements Clock {
  /**
   * Returns the singleton instance for this class.
   *
   * @return singleton instance
   */
  public static WpiClock instance() {
    return _instance;
  }

  /**
   * {@inheritDoc}
   *
   * @return seconds relative to the boot time.
   */
  @Override
  public double nowSecs() {
    return nowMicros() * kSecsPerMicro;
  }

  /**
   * {@inheritDoc}
   *
   * @return milliseconds relative to the boot time.
   */
  @Override
  public double nowMillis() {
    return nowMicros() * kSecsPerMilli;
  }

  /**
   * {@inheritDoc}
   *
   * @return microseconds relative to the boot time.
   */
  @Override
  public long nowMicros() {
    return HALUtil.getFPGATime();
  }

  /**
   * An approximation of how many seconds are left in this period.
   *
   * <p>The period is autonomous or teleop. The time is an
   * approximation that is inferred.
   *
   * @return seconds remaining in the period.
   */
  public double matchSecondsRemainingInPeriod() {
    return Timer.getMatchTime();
  }

  /**
   * The number of microseconds since the Unix Epoch.
   *
   * <p>This is to get the actual date/time if needed. It would be rare
   * for the robot to want to know this. Use {@link #nowMicros} instead
   * for any computation. The {@link #nowMicros} is monotonicly advancing.
   * The system time can move backwards as the internal clock drifts and
   * adjusts itself.
   *
   * @return microseconds since the unix epoch (January 1st, 1970 00:00 UTC).
   */
  public long epochMicros() {
    return edu.wpi.first.util.WPIUtilJNI.getSystemTime();
  }

  /**
   * Singleton constructor.
   */
  private WpiClock() {}

  /**
   * The singleton instance for this class.
   */
  private static final WpiClock _instance = new WpiClock();
}
