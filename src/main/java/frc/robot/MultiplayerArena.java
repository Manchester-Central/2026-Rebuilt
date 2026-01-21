package frc.robot;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class MultiplayerArena<M> {
  private final Arena2026Rebuilt m_arena;
  public static final MultiplayerArena Instance = new MultiplayerArena();

  private enum MatchState {
    WAITING,
    AUTONOMOUS,
    TELEOP,
    COMPLETED
  };

  private MatchState m_matchState;
  private MatchState m_matchStartMode;

  private MultiplayerArena(MatchState startMode) {
    m_arena = new Arena2026Rebuilt(false);
    m_matchState = MatchState.WAITING;
    m_matchStartMode = startMode;
  }

  public void startMatch() {
    m_matchState = m_matchStartMode;
  }
}
