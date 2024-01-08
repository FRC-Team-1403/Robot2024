package frc.lib.device.wpi;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.PowerDistribution;

import frc.lib.device.CurrentSensor;
import frc.lib.device.PowerDistributor;

/**
 * Adapts the WPI PowerDistribution as a CougarLib PowerDistrbutor.
 *
 * <p>The WPI lib supports both CTRE and REV panels, however we use
 * CTRE so only CTRE is explicitly supported.
 */
@SuppressWarnings("PMD.GodClass")
public class WpiPowerDistribution extends PowerDistribution
                                  implements PowerDistributor {
  /**
   * Construct a PowerDistribututor
   *
   * <p>This assumes the PDP is on the default channel kDefaultChannel.
   *
   * @param name The name for the device.
   */
  public WpiPowerDistribution(String name) {
    m_name = name;
  }

  /**
   * Convert faults to a string.
   *
   * <p>This is not part of the interface because there should be
   * a cleaner way to manipulate faults but we don't need that yet.
   *
   * @param faults From getFaults()
   * @return Empty string if no faults.
   */
  public String toFaultString(PowerDistributionFaults faults) {
    StringBuilder result = new StringBuilder(64);

    if (faults.Brownout) {
      result.append("Brownout ");
    }
    if (faults.CanWarning) {
      result.append("CanWarning ");
    }
    if (faults.HardwareFault) {
      result.append("HardwareFault ");
    }

    int circuitBits = breakerFaultBits(faults, m_breakerBits);
    if (circuitBits != 0) {
      for (var it : m_breakerNames.entrySet()) {
        if ((it.getKey().intValue() & circuitBits) != 0) {
          result.append(it.getValue());
          result.append(' ');
        }
      }
    }
    
    return result.toString().trim();
  }

  /**
   * The bitmask used to isolate circuit breaker faults.
   *
   * <p>These are the first 24 bits (0..23). Each circuit is the corresponding bit.
   *
   * <p>See
   * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/hal/
   * PowerDistributionFaults.html
   */
  // First 24 bits (0..23)
  public static final int kCircuitBreakerFaultMask = (0x1 << 24) - 1;

  /**
   * The fault bit denoting a brownout.
   */
  public static final int kBrownoutFaultMask = 0x1 << 24;

  /**
   * The fault bit denoting a CAN warning.
   */
  public static final int kCanFaultMask  = 0x1 << 25;

  /**
   * The fault bit denoting a hardware fault.
   */
  public static final int kHardwareFaultMask  = 0x1 << 26;

  /**
   * Get current faults as a bitmask.
   *
   * @return bitmask of current faults.
   */
  public final int getFaultBits() {
    return getFaultBits(getFaults(), m_breakerBits);
  }

  /**
   * Convert PowerDistributionFaults into a bitmask.
   *
   * @param faults The faults instance to derive bitmask from.
   * @param breakerBits The subset of circuit breaker bits to check.
   * @return bitmask implied by faults
   */
  public static int getFaultBits(PowerDistributionFaults faults,
                                 int breakerBits) {
    return breakerFaultBits(faults, breakerBits)
      | (faults.Brownout ? kBrownoutFaultMask : 0)
      | (faults.CanWarning ? kCanFaultMask : 0)
      | (faults.HardwareFault ? kHardwareFaultMask : 0);
  }

  //--------------------  PowerDistributor Interface  --------------------

  @Override
  public int countCircuits() {
    return getNumChannels();
  }

  @Override
  public CurrentSensor makeCurrentSensor(String name, int channel) {
    if (channel < 0 || channel >= 24) {
      return new CurrentSensor() {
        @Override
        public String getName() {
          return name;
        }

        @Override
        public double getAmps() {
          return 0;
        }
      };
    }

    m_breakerNames.put(0x1 << channel, name);
    m_breakerBits |= 0x1 << channel;

    return new CurrentSensor() {
      @Override
      public String getName() {
        return name;
      }

      @Override
      public double getAmps() {
        return getCurrent(channel);
      }
    };
  }

  @Override
  public double getWatts() {
    return getTotalPower();
  }

  @Override
  public double getTotalJoules() {
    return getTotalEnergy();
  }

  @Override
  public void resetTotalJoules() {
    resetTotalEnergy();
  }
    
  //--------------------  CurrentSensor Interface  --------------------

  @Override
  public final double getAmps() {
    return getTotalCurrent();
  }

  //--------------------  TemperartureSensor Interface  --------------------

  @Override
  public final double getCelsius() {
    return getTemperature();
  }

  //--------------------  Sensor Interface  --------------------

  @Override
  public final String getName() {
    return m_name;
  }

  /**
   * Convert just the circuit breaker fault bits into a bitmask.
   *
   * @param faults The faults to extract breaker fault bits from.
   * @param breakerBits The subset of breaker circuit bits to check.
   * @return bitmask for only breaker faults.
   */
  @SuppressWarnings({"PMD.NcssCount",
                     "PMD.CognitiveComplexity",
                     "PMD.CyclomaticComplexity"})
  public static int breakerFaultBits(PowerDistributionFaults faults,
                                     int breakerBits) {
    int result = 0;

    // It might be simpler / more efficient to just enumerate all
    // the channels rather than iterating.
    int remaining = breakerBits;  // the remaining bit mask not yet checked

    while (remaining != 0) {  // there are still channel (bits) remaining
      // n & n-1 clears least significant bit
      int oneLess = remaining & (remaining - 1);

      // The difference between these is just the least significant bit
      // so xor'ing them will leave just the least significant bit.
      int bit = remaining ^ oneLess;

      // Next iteration has one less channel remaining.
      remaining = oneLess;

      // Update the fault bit for this channel.
      switch (bit) {
        case 0x1 << 0:
          result |= faults.Channel0BreakerFault ? bit : 0;
          break;
        case 0x1 << 1:
          result |= faults.Channel1BreakerFault ? bit : 0;
          break;
        case 0x1 << 2:
          result |= faults.Channel2BreakerFault ? bit : 0;
          break;
        case 0x1 << 3:
          result |= faults.Channel3BreakerFault ? bit : 0;
          break;
        case 0x1 << 4:
          result |= faults.Channel4BreakerFault ? bit : 0;
          break;
        case 0x1 << 5:
          result |= faults.Channel5BreakerFault ? bit : 0;
          break;
        case 0x1 << 6:
          result |= faults.Channel6BreakerFault ? bit : 0;
          break;
        case 0x1 << 7:
          result |= faults.Channel7BreakerFault ? bit : 0;
          break;
        case 0x1 << 8:
          result |= faults.Channel8BreakerFault ? bit : 0;
          break;
        case 0x1 << 9:
          result |= faults.Channel9BreakerFault ? bit : 0;
          break;
        case 0x1 << 10:
          result |= faults.Channel10BreakerFault ? bit : 0;
          break;
        case 0x1 << 11:
          result |= faults.Channel11BreakerFault ? bit : 0;
          break;
        case 0x1 << 12:
          result |= faults.Channel12BreakerFault ? bit : 0;
          break;
        case 0x1 << 13:
          result |= faults.Channel13BreakerFault ? bit : 0;
          break;
        case 0x1 << 14:
          result |= faults.Channel14BreakerFault ? bit : 0;
          break;
        case 0x1 << 15:
          result |= faults.Channel15BreakerFault ? bit : 0;
          break;
        case 0x1 << 16:
          result |= faults.Channel16BreakerFault ? bit : 0;
          break;
        case 0x1 << 17:
          result |= faults.Channel17BreakerFault ? bit : 0;
          break;
        case 0x1 << 18:
          result |= faults.Channel18BreakerFault ? bit : 0;
          break;
        case 0x1 << 19:
          result |= faults.Channel19BreakerFault ? bit : 0;
          break;
        case 0x1 << 20:
          result |= faults.Channel20BreakerFault ? bit : 0;
          break;
        case 0x1 << 21:
          result |= faults.Channel21BreakerFault ? bit : 0;
          break;
        case 0x1 << 22:
          result |= faults.Channel22BreakerFault ? bit : 0;
          break;
        case 0x1 << 23:
          result |= faults.Channel23BreakerFault ? bit : 0;
          break;
        default:
          throw new IllegalArgumentException("Invalid Circuit Id ");
      }
    }
    return result;
  }

  private int m_breakerBits = 0;

  // Maps circuit bit to name.
  // Using TreeMap here so we can traverse in sorted order when reporting.
  private final Map<Integer, String> m_breakerNames = new TreeMap<>();  // NOPMD
  private final String m_name;
}

