package frc.robot.subsystems.MultiplayerSim;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Random;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class MultiplayerArena {
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
  
  // Member Vars
  private final Arena2026 m_arena;
  private Timer phaseTimer;
  private Timer matchTimer;

  private enum MatchState {
    WAITING,
    AUTONOMOUS,
    PHASE_PAUSE,
    TRANSITION_SHIFT,
    TELEOP1,
    TELEOP2,
    TELEOP3,
    TELEOP4,
    END_GAME,
    COMPLETED
  };

  private MatchState m_matchState;
  private MatchState m_matchStartMode;
  private Alliance m_firstAlliance;

  private MultiplayerArena(MatchState startMode) {
    m_arena = new Arena2026(false);
    m_arena.setEfficiencyMode(true);
    m_matchState = MatchState.WAITING;
    m_matchStartMode = startMode;
    m_firstAlliance = null;
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
    if (phaseTimer.get() > DurationAutonomous.in(Seconds)) {
      // Auto is over, tally it up
      int score_diff = m_arena.getScore(Alliance.Blue) - m_arena.getScore(Alliance.Red);
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

  public void periodic() {
    switch (m_matchState) {
      case WAITING:
        // No-op staging environment, waiting for a match to begin
        break;

      case AUTONOMOUS:
        PhaseAutonomous();
        break;

      case PHASE_PAUSE:
        // Not sure, disable the robot somehow?
        // Lets just skip the pause phase and jump to teleop
        // In fact, maybe just start all matches in teleop
        // That's a better idea...
        changePhase(MatchState.TRANSITION_SHIFT);
        break;

      case TRANSITION_SHIFT:
        // Both hubs open
        m_arena.setShouldRunClock(false);
        if (true /* TODO */) {
          //
        }
        break;

      case TELEOP1:
        m_arena.setFirstTeam(m_firstAlliance);
        m_arena.setShouldRunClock(true);
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
        break;

      case COMPLETED:
        // Now sit there quietly
        break;
    }

    // Finally, log the game state
    Logger.recordOutput("Arena/ScoreBlue", m_arena.getScore(Alliance.Blue));
    Logger.recordOutput("Arena/ScoreRed", m_arena.getScore(Alliance.Red));
  }
}
