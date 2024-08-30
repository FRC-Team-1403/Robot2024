package team1403.lib.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

public class CircularSlewRateLimiter {
    private double m_positiveRateLimit;
    private double m_negativeRateLimit;
    private double m_prevVal;
    private double m_prevTime;

    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate
     * limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction,
     *                          in units per
     *                          second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction,
     *                          in units per
     *                          second. This is expected to be negative.
     * @param initialValue      The initial value of the input.
     */
    public CircularSlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        m_positiveRateLimit = positiveRateLimit;
        m_negativeRateLimit = negativeRateLimit;
        m_prevVal = initialValue;
        m_prevTime = MathSharedStore.getTimestamp();
    }

    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and negative
     * rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public CircularSlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }

    /**
     * Filters the input to limit its slew rate. inputs must be angles in radians between [-pi, pi]
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - m_prevTime;
        m_prevVal += MathUtil.clamp(
                MathUtil.angleModulus(input - m_prevVal),
                m_negativeRateLimit * elapsedTime,
                m_positiveRateLimit * elapsedTime);
        m_prevTime = currentTime;
        return m_prevVal;
    }

    public void setLimits(double neg, double pos) {
        m_negativeRateLimit = neg;
        m_positiveRateLimit = pos;
    }

    public void setLimits(double limit) {
        setLimits(-limit, limit);
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
     * Resets the slew rate limiter to the specified value; ignores the rate limit
     * when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = MathSharedStore.getTimestamp();
    }
}
