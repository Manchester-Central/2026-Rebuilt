package frc.robot.subsystems.MultiplayerSim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Random;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.interfaces.AbstractDrive;

public class MultiplayerArena extends Arena2026Rebuilt {
  // Statics
  public static MultiplayerArena Instance;
  public static void init() {
    if (Instance == null) {
      Instance = new MultiplayerArena(MatchState.TRANSITION_SHIFT);
    }
  }
  // Match Phase Times
  public static final Time DurationAutonomous = Seconds.of(20);
  public static final Time DurationPhasePause = Seconds.of(5); 
  public static final Time DurationTransitionShift = Seconds.of(10);
  public static final Time DurationShift1 = Seconds.of(25);
  public static final Time DurationShift2 = Seconds.of(25);
  public static final Time DurationShift3 = Seconds.of(25);
  public static final Time DurationShift4 = Seconds.of(25);
  public static final Time DurationEndGame = Seconds.of(30);
  // match_state_lock
  // match_thread_handler

  // Member Vars
  private Timer phaseTimer;
  private Timer matchTimer;
  private Pose3d winnerAnimationPoint = null;

  // Other Robots
  private RobotContainer[] robots;
  private Pose2d[] startPosition = {
    new Pose2d(Meters.of(-1), Meters.of(0), Rotation2d.kZero),
    new Pose2d(Meters.of(-2), Meters.of(0), Rotation2d.kZero),
    new Pose2d(Meters.of(17), Meters.of(0), Rotation2d.k180deg),
    new Pose2d(Meters.of(18), Meters.of(0), Rotation2d.k180deg),
    new Pose2d(Meters.of(19), Meters.of(0), Rotation2d.k180deg)
  };

  private enum MatchState {
    PREPARE, /* matchRunning == false */
    WAITING,
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
  private MatchState m_matchState;
  private MatchState m_matchStartMode;
  private Alliance m_firstAlliance;
  // TODO: Can we use a thread to drive the Robot state changes from Auto to Teleop?

  private MultiplayerArena(MatchState startMode) {
    super(false);
    phaseTimer = new Timer();
    matchTimer = new Timer();
    setEfficiencyMode(true);
    setShouldRunClock(false);
    m_matchState = MatchState.WAITING;
    m_matchStartMode = startMode;
    // Hard codeded to start with blue alliance by default.
    // It would be nice if we could figure out how to integrate
    // autonomous into this at some point, but I fear that would
    // require an external process to act as FMS.
    m_firstAlliance = Alliance.Blue;
    robots = new RobotContainer[5];
    for (int idx = 0; idx < 5; idx++) {
      // Actual robot container is id 0, rest of player alliance is 1-2
      // opposition is 3-5.
      robots[idx] = new RobotContainer(idx+1, startPosition[idx]);
    }
  }

  public void prepareMatch() {
    resetFieldForAuto();
  }

  public void startMatch() {
    m_matchState = m_matchStartMode;
    matchTimer.restart();
    phaseTimer.restart();
  }

  public void startAutonomous() {
    phaseTimer.restart();
  }

  public void startTeleop() {
    phaseTimer.restart();
  }

  public void changePhase(MatchState newstate) {
    m_matchState = newstate;
    phaseTimer.restart();
  }

  public void PhaseAutonomous() {
    setShouldRunClock(false);
    if (phaseTimer.get() > DurationAutonomous.in(Seconds)) {
      // Auto is over, tally it up
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
      // Finally, ChAAanGe PhAAsEEs!?!
      /*
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
      changePhase(MatchState.PHASE_PAUSE);
    }
  }

  /**
   * NOTE: Not called automatically like a subsystem!
   */
  public void periodic() {
    switch (m_matchState) {
      case PREPARE:
        // No-op staging environment, waiting for a match to begin
        setShouldRunClock(false);
        resetFieldForAuto();
        break;
      case WAITING:
        // No-op
        break;

      case AUTONOMOUS:
        PhaseAutonomous();
        break;

      case PHASE_PAUSE:
        // Not sure, disable the robot somehow?
        // Lets just skip the pause phase and jump to teleop
        // In fact, maybe just start all matches in teleop
        // That's a better idea...
        setShouldRunClock(false);
        changePhase(MatchState.TRANSITION_SHIFT);
        break;

      case TRANSITION_SHIFT:
        // Both hubs open
        setShouldRunClock(false);
        blueIsOnClock = m_firstAlliance == Alliance.Blue;
        break;

      case TELEOP1:
        setShouldRunClock(true);
        // First alliance
        break;

      case TELEOP2:
        // Second alliance
        break;

      case TELEOP3:
        // First Alliance
        break;

      case TELEOP4:
        // Second Alliance
        break;

      case END_GAME:
        // Both teams again!
        setShouldRunClock(true);
        break;

      case COMPLETED:
        // Now sit there quietly
        setWinnerAnimationPoint(getWinner());
        spawnWinnerBall();
        break;
    }

    // Finally, log the game state
    ArenaLogging();
  }

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
   * 
   * @param robot
   * @param releaseVector
   */
  public void launchGamePiece(AbstractDrive robot, Matrix<N3, N1> releaseVector) {
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
            Inches.of(22),
            // The initial speed of the algae
            MetersPerSecond.of(Math.random()*3+2.0),
            // Chooses a random angle between 0 and 60 degrees
            Degrees.of(Math.random()*Math.PI/3)));
  }

  /**
   * Creates a ball in the field for 
   */
  private void spawnWinnerBall() {
    if (winnerAnimationPoint == null) {
      System.out.println("Can't spawn ball without spawn location...");
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
   * Logs overall match and field information.
   */
  public void ArenaLogging() {
    Logger.recordOutput("Arena/ScoreBlue", getScore(Alliance.Blue));
    Logger.recordOutput("Arena/ScoreRed", getScore(Alliance.Red));
    Logger.recordOutput("Arena/GameState", m_matchState);
    Logger.recordOutput("Arena/GamePieces", getGamePiecesArrayByType("Fuel"));
  }
}
