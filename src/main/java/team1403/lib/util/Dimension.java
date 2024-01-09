package team1403.lib.util;

/**
 * Class to store dimensions of an object.
 * Stores 3d dimensions, the 3rd dimension can be null.
 */
public class Dimension {
  private final double m_height;
  private final double m_width;
  private final double m_length;

  /**
   * Dimensions for arm.
   *
   * @param height height of arm
   * @param width width of arm
   */
  public Dimension(double height, double width) {
    this.m_height = height;
    this.m_width = width;
    this.m_length = 0;
  }

  /**
   * Dimensions for arm.
   *
   * @param height height of arm
   * @param width width of arm
   * @param length length of arm
   */
  public Dimension(double height, double width, double length) {
    this.m_height = height;
    this.m_width = width;
    this.m_length = length;
  }

  public double getHeight(double armHeight) {
    return m_height;
  }

  public double getWidth() {
    return m_width;
  }

  public double getLenth() {
    return m_length;
  }

}
