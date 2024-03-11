package team1403.robot.subsystems.arm;

/**
 * Creates the ArmState class.
 * 
 */
public class ArmState {
  public final double armLength;
  public final double wristAngle;
  public final double armPivot;
  public final double intakeSpeed;

  /**
   * Initializes the ArmState class.
   */
  public ArmState(double armLength, double wristAngle, double armPivot,
      double intakeSpeed) {
    this.armLength = armLength;
    this.wristAngle = wristAngle;
    this.armPivot = armPivot;
    this.intakeSpeed = intakeSpeed;
  }

  @Override
  public String toString() {
    return "ArmState [armLength=" + armLength + "]";
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long temp;
    temp = Double.doubleToLongBits(armLength);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(wristAngle);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(armPivot);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(intakeSpeed);
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
    if (Double.doubleToLongBits(armLength) != Double.doubleToLongBits(other.armLength)) {
      return false;
    }
    if (Double.doubleToLongBits(wristAngle) != Double.doubleToLongBits(other.wristAngle)) {
      return false;
    }
    if (Double.doubleToLongBits(armPivot) != Double.doubleToLongBits(other.armPivot)) {
      return false;
    }
    if (Double.doubleToLongBits(intakeSpeed) != Double.doubleToLongBits(other.intakeSpeed)) {
      return false;
    }  
    return true;
  }

}