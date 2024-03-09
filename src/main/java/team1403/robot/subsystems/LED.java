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
    gamePieceState = LEDState.WHITE;

  }

  /* set by the buttons
   * - saves the game state
   * - sets the SD boolean box to color
   * - sets the physical LED
  */
  public void setLedMode(LEDState ledState) {
   this.gamePieceState = ledState;

   blinkin.set(gamePieceState.colorValue);
  }

  public GamePiece getGameState() {
    return this.gamePieceState.gamePiece;
  }

  public enum LEDState {
    WHITE(-0.21, GamePiece.NOTREADY), GREEN(0.77, GamePiece.READY);

    public final double colorValue;
    public final GamePiece gamePiece;

    LEDState(double colorValue, GamePiece gamePiece)  {
        this.colorValue = colorValue;
        this.gamePiece = gamePiece;
    }
  }

  public enum GamePiece {
    READY,
    NOTREADY,
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED Color", gamePieceState.toString());
    Logger.recordOutput("LED State", getGameState());
  }
}
