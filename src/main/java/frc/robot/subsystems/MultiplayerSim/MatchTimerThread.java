package frc.robot.subsystems.MultiplayerSim;

import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.constants.ArenaConstants;
import frc.robot.subsystems.MultiplayerSim.MultiplayerArena2026.MatchState;

/**
 * Tightly coupled thread timer with the MultiplayerArena2026 class because
 * both share the same phase structure and logic. No good way to decouple
 * without treating as an enclosed class, or creating many wrapper functions.
 */
public class MatchTimerThread extends Thread {
  protected Timer phases_timer;
  protected boolean is_running;
  protected MultiplayerArena2026 arena;
  protected MatchAudio audio;
  protected boolean abortFlag;

  // Match Phase Times
  public static final Time DurationPrepare = Seconds.of(3);
  public static final Time DurationAutonomous = Seconds.of(20);
  public static final Time DurationPhasePause = Seconds.of(5); 
  public static final Time DurationTransitionShift = Seconds.of(10);
  public static final Time DurationShift1 = Seconds.of(25);
  public static final Time DurationShift2 = Seconds.of(25);
  public static final Time DurationShift3 = Seconds.of(25);
  public static final Time DurationShift4 = Seconds.of(25);
  public static final Time DurationEndGame = Seconds.of(30);
  public static final Time DurationCelebration = Seconds.of(10);

  public MatchTimerThread(MultiplayerArena2026 arena) {
    super();
    this.arena = arena;
    this.audio = MatchAudio.getInstance();
    phases_timer = new Timer();
    is_running = false;
    abortFlag = false;
  }

  public void abort() {
    abortFlag = true;
  }

  @Override
  public void run() {
    try {
      this.arena.resetFieldForAuto();
      setPrepare();
      Thread.sleep((long)DurationPrepare.in(Millisecond));
      
      if (ArenaConstants.matchStartState == MatchState.AUTONOMOUS) {
        setAutonomous();
        Thread.sleep((long)DurationAutonomous.in(Millisecond));

        setIntermission();
        Thread.sleep((long)DurationPhasePause.in(Millisecond));
      }

      setTeleop();
      Thread.sleep((long)DurationTransitionShift.in(Millisecond));
      setTeleopPhase(MatchState.TELEOP1);
      Thread.sleep((long)DurationShift1.in(Millisecond));
      setTeleopPhase(MatchState.TELEOP2);
      Thread.sleep((long)DurationShift2.in(Millisecond));
      setTeleopPhase(MatchState.TELEOP3);
      Thread.sleep((long)DurationShift3.in(Millisecond));
      setTeleopPhase(MatchState.TELEOP4);
      Thread.sleep((long)DurationShift4.in(Millisecond));
      setEndGame();
      Thread.sleep((long)DurationEndGame.in(Millisecond));

      matchOver();
      Thread.sleep((long)DurationCelebration.in(Millisecond));
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  /**
   * We're ignoring half cycles and phase accumulation error here,
   * technically this is irresponsible but I'm also lazy.
   */
  private boolean survivesPhase(Time phase_dur) {
    Timer timer = new Timer();
    timer.start();
    while (!timer.hasElapsed(phase_dur)) {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
      }
    }
    return true;
  }

  /**
   * Occurs during the countdown for a match
   */
  private void setPrepare() {
    // TODO: Play Countdown sound.
    this.arena.changePhase(MatchState.PREPARE);
    audio.playAudioFromStart(MatchAudio.START, 1.0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private void setAutonomous() {
    this.arena.changePhase(MatchState.AUTONOMOUS);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  private void setIntermission() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private void setTeleop() {
    // We do this at the start of teleop for datarace and simplicity's sake.
    this.arena.calculateAutoWinner();
    audio.playAudioFromStart(MatchAudio.RESUME, 1.0);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  private void setTeleopPhase(MatchState phase) {
    this.arena.changePhase(phase);
  }

  private void setEndGame() {
    this.arena.changePhase(MatchState.END_GAME);
    audio.playAudioFromStart(MatchAudio.ENDGAME, 1.0);
  }

  private void matchOver() {
    this.arena.changePhase(MatchState.COMPLETED);
    audio.playAudioFromStart(MatchAudio.WINNER, 1.0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }
}
