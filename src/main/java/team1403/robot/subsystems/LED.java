package team1403.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import team1403.lib.util.CougarLogged;
import team1403.robot.Constants;

public class LED extends SubsystemBase implements CougarLogged {

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



  public static enum LEDState {
    OFF(0.99), YELLOW(0.69), GREEN(0.75), YELLOW_FLASH(-0.07), RED_FLASH(-0.17), DARK_RED(0.59), DARK_BLUE(0.85), 
    RAINBOW_FOREST(-0.91), RAINBOW(-0.99);

    public final double colorValue;

    LEDState(double colorValue) {
      this.colorValue = colorValue;
    }

  }

  @Override
  public void periodic() {
    log("LED State", gamePieceState.toString());
  }
}
