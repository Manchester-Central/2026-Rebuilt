package frc.robot.subsystems.MultiplayerSim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.Random;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ArenaConstants;

public class MultiplayerArena2026 extends Arena2026Rebuilt {
  // Statics
  public static MultiplayerArena2026 Instance;
  static {
    Instance = new MultiplayerArena2026();
    SimulatedArena.overrideInstance(Instance);
  }
  protected MatchTimerThread timerThread;
  private ArrayList<LoggedDashboardChooser<Runnable>> arenaEnable = new ArrayList<>();

  public static enum MatchState {
    WAITING,
    PREPARE, /* matchRunning == false */
    AUTONOMOUS, /* matchRunning == true */
    PHASE_PAUSE,
    TRANSITION_SHIFT,
    TELEOP1,
    TELEOP2,
    TELEOP3,
    TELEOP4,
    END_GAME,
    COMPLETED /* matchRunning == false */
  };
  private class MatchStartCommand extends Command {
    @Override
    public void initialize() {
      startMatch();
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }
  private class MatchAbortCommand extends Command {
    @Override
    public void initialize() {
      abortMatch();
    }

    @Override
    public boolean runsWhenDisabled(){
      return true;
    }
  }
  private MatchStartCommand startButton;
  private MatchAbortCommand abortButton;
  private MatchState m_matchState;
  private boolean runAutonomous;
  private boolean runTeleop;

  // Scoring
  private Alliance m_firstAlliance;
  private Pose3d winnerAnimationPoint = null;
  private Pose3d redScoringIndicator = new Pose3d(Inches.of(650.12-156.06), Inches.of(158.84), Inches.of(44.25), new Rotation3d());
  private Pose3d blueScoringIndicator = new Pose3d(Inches.of(156.06), Inches.of(158.84), Inches.of(44.25), new Rotation3d());

  // Other Robots
  private RobotContainer[] robots;


  private MultiplayerArena2026() {
    super(false);
    setEfficiencyMode(true);
    setShouldRunClock(false);
    m_matchState = MatchState.WAITING;
    Logger.recordOutput("Arena/GameState", m_matchState);
    runAutonomous = true;
    runTeleop = true;

    m_firstAlliance = Alliance.Blue;

    startButton = new MatchStartCommand();
    SmartDashboard.putData("Arena/StartMatch", startButton);
    abortButton = new MatchAbortCommand();
    SmartDashboard.putData("Arena/AbortMatch", abortButton);
  }

  /**
   * Two phase init that should be run after the primary RobotContainer is created.
   * This is because the primary robot has special permissions and static initializations.
   *
   * There's a lot of work in here that normally should exist in other parts of the code,
   * but we're trying to remove any Arena code from other robot behavior code.
   */
  @SuppressWarnings("unused")
  public void loadAdditionalRobots() {
    if (ArenaConstants.numAdditionalRobots > 5)
      throw new RuntimeException("Too many additional robots declared, max number is 5!"); 
    
    robots = new RobotContainer[ArenaConstants.numAdditionalRobots];
    for (int idx = 0; idx < ArenaConstants.numAdditionalRobots; idx++) {
      // Actual robot container is id 0, rest of player alliance is 1-2
      // opposition is 3-5.
      robots[idx] = new RobotContainer(idx+1, ArenaConstants.startingPoses[idx+1]);
      arenaEnable.add(addMultiplayerChooser(idx+1));
    }
    // Also add the phase toggles to the 
    arenaEnable.add(setupAutoToggle());
    arenaEnable.add(setupTeleopToggle());
  }

  private LoggedDashboardChooser<Runnable> setupAutoToggle() {
    var auto = new SendableChooser<Runnable>();
    final Runnable disable = () -> runAutonomous = false;
    final Runnable enable = () -> runAutonomous = true;
    auto.setDefaultOption("Disable", disable);
    auto.addOption("Enable", enable);
    return new LoggedDashboardChooser<>("Arena/runAutonomous", auto);
  }

  private LoggedDashboardChooser<Runnable> setupTeleopToggle() {
    var teleop = new SendableChooser<Runnable>();
    final Runnable disable = () -> runAutonomous = false;
    final Runnable enable = () -> runAutonomous = true;
    teleop.setDefaultOption("Disable", disable);
    teleop.addOption("Enable", enable);
    return new LoggedDashboardChooser<>("Arena/runTeleop", teleop);
  }

  public void prepareMatch() {
    resetFieldForAuto();
  }

  /**
   * Triggered by the startButton / MatchStartCommand
   */
  private void startMatch() {
    synchronized(this) {
      if (timerThread == null) {
        System.out.println("[DEBUG] Starting Match!");
        timerThread = new MatchTimerThread(this);
        timerThread.start();
      }
    }
  }

  private void abortMatch() {
    synchronized(this) {
      if (timerThread != null) {
        System.out.println("[DEBUG] Aborting Match!");
        timerThread.interrupt();
      }
    }
  }

  /**
   * Runs ever Arena loop, checks if it's done, and destroys the handler if so
   */
  private void matchThreadCleanup() {
    synchronized(this) {
      if (timerThread != null && !timerThread.isAlive()) {
        timerThread = null;
      }
    }
  }

  public void changePhase(MatchState newstate) {
    /* ChAAanGe PhAAsEEs!?!
                      .,,;;;;;!!!!!!!!!;;;,.
                .,;;!!!!!!!!!!!!!!!!!!!!!!!!!!!;
          .,;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!>
      ,;;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!>
    ,;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
  ,<!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
,<!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!>
;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!>
<!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!>''''``''
<!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!''` ....:::::::::
;!!!!!!!!!!!!!!!!!!!'`!!!!!!!!''`...:::::::::::::::::
!!!!!!!!!!!!!!!!!' ,z `!!!'` .:::::::::::::::::::::::
``'!!!!!!!!!!!'',c$$$b ` .::::::::::::::::::::::::::::
`'!!!!!' z$$$ $$$b `:::::::::::::::::::::::::::::
    `!' c$$$$$h`$$$$ `:::::::::::::::::::::::::::::     _,;;;
      z$P"_"?$$ $$"$$. `::::::::::::::::::::::::::::  '` .. `!
    z$$$ $$$c ",`F ,"?c,`::'''```````''':::::::::::::   <!' <!
  J$?$$,`?$$$ 3 $c,,d$P" ,;;!!!!!!!!!!;;,.```'::::::::   ,!'
  d$$,,`?bc,"" Jh`$$P",;!!!!!!!!!!!!!!!!!!!!!!;;;,,,,,,,;'`,nPP"""=-
`"""??$-` .,nmnn,.  `~h '``!!!!!!!!!!!!!!!''`````````   =MMMMMMMMMbn
      cdMMMMMMMMMMMMn, ` "n.`<!!!!!!!' ,-=~"??$$"$P"     `MMMP"   `""
        .,n,."?MMMMMMMMb,  MMb  `!'`,c$"        "?" ,cc$c ,MMMMMMP4x
    ,nMMMMMMMb,3MMMMMMMM, dMMr   c$$$F ,cd$$$?$ccd$$P??? MMMMMMMM,"b
  nMMMMMMMMMMMMMMMMMMMMMMMMMM> h ?$$$$$$$$$"<$$c`$$'cd$, "MMMMMP4M M
uMMMMMMMMMMMMMMMMMMMMMMMMMMMMLM>. "$$$$$$$$.`$"  $$c   ",= """T' '
JP MMMMP MMMMMMMMMMMMMMMMMMMMMMM.M> $$$$$$$$$c,,,c$$$$bc,cc$$$b,
' dMMM",MMMMMMMMMMMMMMMMMMMMMP""""" ?$$F<$$$$$$$$$$P???$$$$$$$$$
dMP" ,MMMMMMMMMMMMMMMMMMMMM zJF"??cc,` $$$$$$$$$$$ $bcJ$$$$$$$P
M    MMMMMMMMMMMMMMMMMMMMM:<$$$$c  "$ J$$$$$$$PF" ."$$$$$$$$P" .
    4M4MMMMM4MMMMMMMMMMMMML ?$$$$ <c, $$$$?$P ,cd$$cc,"""   cc$$r
    `M'MMMM>4MMMMMMMMMMMMMMx "$$$b,"" $$$$ $ '$$$$$$$$$$ 4$,"$$$F
        MMMM> MMMMMMMM""?MM> "?n `"??? $$$$ ? -`?$$$$$$$$c`?$c$$$F
        "MML ."?MMMM     ""       "$$.`""$bJ.4  "?$$$$$$F  $$$$$'
          "?< ?   ""-              ,;;!> $$$$ b "cc ,, c $$$$$$F  ,,-
                                ,;!!!!!! `$$$c`b."$ $$ $ "$$$$$',!!!
                              ,;!!!!!!!!!! `$$$$$$, "====",$$$$',!!'
                            ,<!!!!!!!!!!!!> `$$$$$$$$ccc$$$F,$',!!
                          ``<!!!!!!!!!!!!!!.`??$$$$$$$$$$"," ;!'
                              ``'!!!!!!!!!!! ;; "$$$$$$P" ,;>
                                  `<!!!!!!!';!!! `$$$P" ;!!!!>
                                      .,.`` `!!!!  "" -''!'' ,!
                                    ,!!!!!!!>;, `'\ <!>   ,;<!!!
      */
    switch (m_matchState) {
      case WAITING:
      case PREPARE:
      case AUTONOMOUS:
      case TRANSITION_SHIFT:
      case PHASE_PAUSE:
      case END_GAME:
        setShouldRunClock(false);
        break;

      case COMPLETED:
        setShouldRunClock(false);
        setWinnerAnimationPoint(getWinner());
        break;

      case TELEOP1:
      case TELEOP2:
      case TELEOP3:
      case TELEOP4:
        setShouldRunClock(true);
        break;
    }

    m_matchState = newstate;
    Logger.recordOutput("Arena/GameState", m_matchState);
  }

  /**
   * Should only be called from the MatchTimerThread, because that's what schedules
   * the phase changes, not this class.
   */
  public void calculateAutoWinner() {
    int score_diff = getScore(Alliance.Blue) - getScore(Alliance.Red);
    if (score_diff == 0) {
      // If the score is even after Auto, we pick a random team
      if (new Random().nextBoolean()) {
        m_firstAlliance = Alliance.Blue;
      } else {
        m_firstAlliance = Alliance.Red;
      }
    } else if (score_diff > 0) {
      m_firstAlliance = Alliance.Blue;
    } else {
      m_firstAlliance = Alliance.Red;
    }
    blueIsOnClock = m_firstAlliance == Alliance.Blue;
  }

  /**
   * NOTE: Not called automatically like a subsystem!
   */
  public void periodic() {
    switch (m_matchState) {
      case PREPARE:
      case WAITING:
      case PHASE_PAUSE:
      case AUTONOMOUS:
      case TRANSITION_SHIFT:
      case TELEOP1:
      case TELEOP2:
      case TELEOP3:
      case TELEOP4:
      case END_GAME:
        break;
      case COMPLETED:
        // Now sit there quietly
        spawnWinnerBall();
        break;
    }

    // Do the game tick
    simulationPeriodic();

    // Finally, log the game state
    arenaLogging();
  }

  /**
   * Helper function to indicate if we're in the alternative phase section of a match.
   * This is critical to support isActive because isActive builds on the 
   * @return true if in one of the 4 alternative phases
   */
  private boolean isTeamPhase() {
    switch (m_matchState) {
      case TELEOP1:
      case TELEOP2:
      case TELEOP3:
      case TELEOP4:
        return true;
      
      default:
        return false;
    }
  }

  /**
   * Custom function to handle if a hub is active or not 
   */
  @Override
  public boolean isActive(boolean isBlue) {
    if (isBlue) {
      return (blueIsOnClock && isTeamPhase()) || DriverStation.isAutonomous() || !DriverStation.isEnabled();
    } else {
      return (!blueIsOnClock && isTeamPhase()) || DriverStation.isAutonomous() || !DriverStation.isEnabled();
    }
  }

  /**
   * Determines which alliance won at the end of the event
   * @return The alliance enum, or null if it's a tie!
   */
  private Alliance getWinner() {
    if (getScore(Alliance.Blue) > getScore(Alliance.Red)) {
      return Alliance.Blue;
    } else if (getScore(Alliance.Red) > getScore(Alliance.Blue)) {
      return Alliance.Red;
    }
    return null;
  }

  private void setWinnerAnimationPoint(Alliance winningAlliance) {
    if (winningAlliance == Alliance.Blue) {
      winnerAnimationPoint = new Pose3d(0, 4, 2.0, new Rotation3d());
    } else if (winningAlliance == Alliance.Red) {
      winnerAnimationPoint =  new Pose3d(16, 4, 2.0, new Rotation3d());
    } else {
      winnerAnimationPoint = null;
    }
  }

  /**
   * Creates a ball in the field for 
   */
  private void spawnWinnerBall() {
    if (winnerAnimationPoint == null) {
      // System.out.println("Can't spawn ball without spawn location...");
      return;
    }

    addGamePieceProjectile(new RebuiltFuelOnFly(
            // Obtain robot position from drive simulation
            winnerAnimationPoint.getTranslation().toTranslation2d(),
            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
            new Translation2d(0.1, 0.0),
            // Obtain robot speed from drive simulation
            new ChassisSpeeds(),
            // Release the fuel +/- 60 degrees from forward, remember to flip for red
            new Rotation2d((2*Math.random()-1)*Math.PI/3 + (getWinner() == Alliance.Red ? 180 : 0)),
            // The height at which the algae is ejected
            Meters.of(2.0),
            // The initial speed of the algae
            MetersPerSecond.of(Math.random()*3+2.0),
            // Chooses a random angle between 0 and 60 degrees
            Degrees.of(Math.random()*Math.PI/3)));
  }

  /**
   * This is a function that should probably be in RobotContainer, but it's isolated into a helper function here to
   * keep the RobotContainer free of changes that could be done somewhere else to reduce the chances of merge conflicts.
   *
   * @param robotId - 0 for original robot, max id of 5
   * @return
   */
  private LoggedDashboardChooser<Runnable> addMultiplayerChooser(int robotId) {
    var tmp = new SendableChooser<Runnable>();
    final Runnable disable = () -> robots[robotId].getSwerveDrive().setPose(ArenaConstants.waitingPoses[robotId]);
    final Runnable enable = () -> robots[robotId].getSwerveDrive().setPose(ArenaConstants.startingPoses[robotId]);
    tmp.setDefaultOption("Disable", disable);
    tmp.addOption("Enable", enable);

    return new LoggedDashboardChooser<>("Robot"+robotId+"/Enabled", tmp);
  }

  /**
   * Logs overall match and field information.
   */
  public void arenaLogging() {
    for (RobotContainer container : robots) {
      container.containerLogging();
    }
    matchThreadCleanup();

    Logger.recordOutput("Arena/ThreadActive", timerThread != null);
    Logger.recordOutput("Arena/BlueActive", isActive(true) ? new Pose3d[]{blueScoringIndicator} : new Pose3d[0]);
    Logger.recordOutput("Arena/BlueScore", getScore(Alliance.Blue));
    Logger.recordOutput("Arena/RedActive", isActive(false) ? new Pose3d[]{redScoringIndicator} : new Pose3d[0]);
    Logger.recordOutput("Arena/RedScore", getScore(Alliance.Red));
    // Logger.recordOutput("Arena/GameState", m_matchState);
    Logger.recordOutput("Arena/GamePieces", getGamePiecesArrayByType("Fuel"));
  }
}
