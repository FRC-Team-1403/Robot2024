package team1403.lib.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class ContinuousSlewRateLimiter {
  private final double m_positiveRateLimit;
  private final double m_negativeRateLimit;
  private double m_prevVal;
  private double m_prevTime;
  private double m_minInput;
  private double m_maxInput;

  /**
   * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
   * value.
   *
   * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
   *     second. This is expected to be positive.
   * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
   *     second. This is expected to be negative.
   * @param initialValue The initial value of the input.
   */
  public ContinuousSlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue, double minInput, double maxInput) {
    m_positiveRateLimit = positiveRateLimit;
    m_negativeRateLimit = negativeRateLimit;
    m_prevVal = initialValue;
    m_prevTime = MathSharedStore.getTimestamp();
    m_minInput = minInput;
    m_maxInput = maxInput;
  }

  /**
   * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
   * -rateLimit.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public ContinuousSlewRateLimiter(double rateLimit, double minInput, double maxInput) {
    this(rateLimit, -rateLimit, 0, minInput, maxInput);
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - m_prevTime;
    double errorBound = (m_maxInput - m_minInput) / 2.0;
    m_prevVal +=
        MathUtil.clamp(
            MathUtil.inputModulus(input - m_prevVal, -errorBound, errorBound),
            m_negativeRateLimit * elapsedTime,
            m_positiveRateLimit * elapsedTime);
    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * Returns the value last calculated by the SlewRateLimiter.
   *
   * @return The last value.
   */
  public double lastValue() {
    return m_prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = MathSharedStore.getTimestamp();
  }
}
