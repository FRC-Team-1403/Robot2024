package team1403.robot.subsystems;

/**
 * Creates the ArmState class.
 * 
 */
public class ArmState {
  public final double armPivot;

  /**
   * Initializes the ArmState class.
   */
  public ArmState(double armPivot) {
    this.armPivot = armPivot;

  }
  
  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long temp;
    temp = Double.doubleToLongBits(armPivot);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    ArmState other = (ArmState) obj;
    if (Double.doubleToLongBits(armPivot) != Double.doubleToLongBits(other.armPivot)) {
      return false;
    }
    return true;
  }

}