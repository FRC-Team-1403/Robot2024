package team1403.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team1403.robot.Constants.ArmStates;
import team1403.robot.subsystems.arm.ArmStateGroup;

public class StateManager {

  private Optional<Alliance> m_alliance;

  private ArmStateGroup m_currentArmGroup;
  private ArmStateGroup m_cubeGroup;
  private ArmStateGroup m_coneUprightGroup;
  private ArmStateGroup m_coneTowardsGroup;
  private ArmStateGroup m_coneAwayGroup;  private int cubeCounter = 1;
  private int coneTowardsCounter = 1;
  private int coneAwayCounter = 1;
  private int gamePieceCounter = 1;

  private int m_armGroupUsed = 0;

  private int counter = 0;

  private GamePiece gamePiece = GamePiece.NONE;
  private LED led = LED.NONE;

  public enum GamePiece {
    CUBE,
    CONE_UPRIGHT,
    CONE_AWAY, 
    CONE_SIDEWAYS, 
    CONE_TOWARDS,
    NONE;

    static final GamePiece[] values = GamePiece.values();

    public static GamePiece fromInt(int value) {
      switch(value) {
        case 4: // a cube has 4 sides :p
          return CUBE;
        case 117: // 'u'
          return CONE_UPRIGHT;
        case 115: // 's'
          return CONE_SIDEWAYS;
        case 104: // 'h'
          return CONE_TOWARDS;
        case 97:  // 'a'
          return CONE_AWAY;
        default: // nothing's there :O
          return NONE;
      }
    }
  }

  public enum LED {
    PURPLE,
    YELLOW,
    MONTY,
    RAINBOW,
    NONE;
  }

  private static StateManager instance = new StateManager();

  private StateManager() {
    m_coneTowardsGroup = new ArmStateGroup(ArmStates.coneTowardsFloorIntake, null, 
        ArmStates.singleSubstationIntake, ArmStates.coneTowardsHighConeNode, 
        ArmStates.coneTowardsMiddleNode, ArmStates.coneTowardsLowNode);

    m_cubeGroup = new ArmStateGroup(ArmStates.cubeFloorIntake, null, 
        ArmStates.singleSubstationIntake, ArmStates.cubeHighNode, 
        ArmStates.cubeMiddleNode, ArmStateGroup.tuck);

    m_coneUprightGroup = new ArmStateGroup(ArmStates.coneUprightIntake, null, 
        ArmStates.singleSubstationIntake, ArmStates.coneTowardsHighConeNode, 
        ArmStates.coneTowardsMiddleNode, ArmStates.coneTowardsLowNode);

    m_currentArmGroup = m_coneTowardsGroup;
  }

  public static StateManager getInstance() {
    return instance;
  }

  public void init() {
    m_alliance = DriverStation.getAlliance();
  }

  public void updateArmState(GamePiece newGamePiece) {
    gamePiece = newGamePiece;
    SmartDashboard.putString("Game Piece", newGamePiece.toString());
    if (newGamePiece == GamePiece.CONE_UPRIGHT) {
      m_currentArmGroup = m_coneUprightGroup;
      m_armGroupUsed = 0;
      coneAwayCounter++;
      updateLEDState(LED.YELLOW);
    } else if (newGamePiece == GamePiece.CUBE) {
      m_currentArmGroup = m_cubeGroup;
      m_armGroupUsed = 1;
      cubeCounter++;
      updateLEDState(LED.PURPLE);
    } else if (newGamePiece == GamePiece.CONE_TOWARDS) {
      m_currentArmGroup = m_coneTowardsGroup;
      m_armGroupUsed = 2;
      coneTowardsCounter++;
      updateLEDState(LED.YELLOW);
    }
  }

  public void updateLEDState(LED newLEDState) {
    this.led = newLEDState;
  }

  public LED getLEDState() {
    return led;
  }

  public Optional<Alliance> getalliance() {
    return m_alliance;
  }

  public ArmStateGroup getCurrentArmGroup() {
    SmartDashboard.putString("Expected group", m_currentArmGroup.getHighNodeState().toString());
    counter++;
    if (m_armGroupUsed == 0) {
      return m_coneUprightGroup;
    } else if (m_armGroupUsed == 1) {
      return m_cubeGroup;
    } else {
      return m_coneTowardsGroup;
    }
  }

  public GamePiece getGamePiece() {
    return gamePiece;
  }

  public ArmStateGroup getCubeGroup()
  {
    return m_cubeGroup;
  }

  public ArmStateGroup getUprightConeGroup()
  {
    return m_coneUprightGroup;
  }

}
    
