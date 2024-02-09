package team1403.robot.subsystems.arm;

/**
 * Creates the ArmState class.
 * 
 */
public class ArmState {
  public final double armPivot;
  public final double wristAngle;

  /**
   * Initializes the ArmState class.
   */
  public ArmState(double armPivot, double wrist) {
    this.armPivot = armPivot;
    this.wristAngle = wrist;

  }
  
  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long temp;
    temp = Double.doubleToLongBits(armPivot);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(wristAngle);
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
    if (Double.doubleToLongBits(wristAngle) != Double.doubleToLongBits(other.wristAngle)) {
      return false;
    }
    return true;
  }

}