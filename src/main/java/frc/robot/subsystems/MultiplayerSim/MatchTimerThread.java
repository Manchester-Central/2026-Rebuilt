package frc.robot.subsystems.MultiplayerSim;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

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

  // Match Phase Times
  public static final Time DurationAutonomous = Seconds.of(20);
  public static final Time DurationPhasePause = Seconds.of(5); 
  public static final Time DurationTransitionShift = Seconds.of(10);
  public static final Time DurationShift1 = Seconds.of(25);
  public static final Time DurationShift2 = Seconds.of(25);
  public static final Time DurationShift3 = Seconds.of(25);
  public static final Time DurationShift4 = Seconds.of(25);
  public static final Time DurationEndGame = Seconds.of(30);

  public MatchTimerThread(MultiplayerArena2026 arena) {
    super();
    this.arena = arena;
    phases_timer = new Timer();
    is_running = false;
  }

  @Override
  public void run() {
    phases_timer.restart();
    while (is_running) {
      setAutonomous();
      is_running = false;
    }
  }

  private void setAutonomous() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  private void setIntermission() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private void setTeleop() {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }
}
