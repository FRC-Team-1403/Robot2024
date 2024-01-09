package team1403.lib.util.test;

import java.util.PrimitiveIterator;
import java.util.stream.DoubleStream;

import team1403.lib.util.Clock;


/**
 * FakeClock for testing purposes.
 *
 * <p>This can be used in 3 ways:
 *    * Advance by constant every access
 *      {@link #FakeClock(double,double)}
 *    * Advance by explicit sequence each access
 *      {@link #FakeClock(double[])}
 *    * Explicitly advance yourself
 *      {@link #FakeClock(double,double)}
 *      and {@link #advanceSecs}
 */
public class FakeClock implements Clock {
  /**
   * Construct clock at constant tempo.
   *
   * @param start Initial start time is the first now() value.
   * @param delta Incremental time for each now() thereafter.
   */
  public FakeClock(double start, double delta) {
    if (start < 0 || delta < 0) {
      // We'll throw an exception since this is test-only code.
      throw new IllegalArgumentException(
          String.format(
              "Cannot advance negative start or delta (start=%f, delta=%f)",
              start, delta));
    }
    m_delta = delta;
    m_now = start - delta;
  }

  /**
   * Construct clock for at specific times.
   *
   * @param times list of times to return for now().
   */
  public FakeClock(double... times) {
    if (times.length > 0) {
      m_iterator = DoubleStream.of(times).iterator();
    }
  }

  /**
   * Advance the clock.
   *
   * <p>If using this call then you should construct with delta=0.
   * Otherwise the clock will still continue to advance automatically
   * as you check its time.
   *
   * @param delta Seconds to advance by.
   */
  public void advanceSecs(double delta) {
    if (delta < 0) {
      // We'll throw an exception since this is test-only code.
      throw new IllegalArgumentException(
          String.format("Cannot advance negative time (delta=%f)", delta));
    }
    m_now += delta;
  }

  @Override
  public double nowSecs() {
    if (m_iterator != null) {
      if (!m_iterator.hasNext()) {
        throw new IllegalStateException("Exceeded fake clock times");
      }
      return m_iterator.next();
    }
    m_now = m_now + m_delta;
    return m_now;
  }

  @Override
  public double nowMillis() {
    return nowSecs() * 1000;
  }

  @Override
  public long nowMicros() {
    return (long)(nowSecs() * 1000000);
  }

  private double m_now;
  private double m_delta;
  private PrimitiveIterator.OfDouble m_iterator;
}
