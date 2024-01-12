package team1403.lib.device;

/**
 * Denotes that the program requested a device that does not exist.
 *
 * <p>This is an unchecked exception because it is a programming error.
 * We throw these exceptions to catch these programming errors early.
 * It does not normally make sense to try to catch these errors because
 * they are bugs which should be fixed at their source.
 */
public class NoSuchDeviceError extends RuntimeException {
  /**
   * Constructor.
   *
   * @param message Describes what device was wanted.
   */
  public NoSuchDeviceError(String message) {
    super(message);
  }

  static final long serialVersionUID = 1L;
}
