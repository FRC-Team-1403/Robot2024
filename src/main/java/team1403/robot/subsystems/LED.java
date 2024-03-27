package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class LED extends SubsystemBase {

  private Spark blinkin;
  private LEDState gamePieceState;
  /* Creates a new LED. */
  public LED() {
    this.blinkin = new Spark(Constants.RioPorts.LEDPort);
    gamePieceState = LEDState.OFF;
  }

  /* set by the buttons
   * - saves the game state
   * - sets the SD boolean box to color
   * - sets the physical LED
  */
  public void setLedMode(LEDState ledState) {
   this.gamePieceState = ledState;

   setLedColor(gamePieceState.colorValue);
  }

  public void setLedColor(double color) {
    blinkin.set(color);
  }

  public enum LEDState {
    OFF(0.99), YELLOW(0.69), GREEN(0.75), YELLOW_FLASH(-0.07), RED_FLASH(-0.17);

    public final double colorValue;

    LEDState(double colorValue) {
      this.colorValue = colorValue;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED Color", gamePieceState.toString());
    Logger.recordOutput("LED State", gamePieceState.toString());
  }
}
