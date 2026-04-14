package frc.robot.subsystems.interfaces;

public abstract class AudioInterface {
  public static AudioInterface instance;

  public static AudioInterface getInstance() {
    return instance;
  }

  public static final String START = "audio/start.wav";
  public static final String RESUME = "audio/resume.wav";
  public static final String ENDGAME = "audio/warning.wav";
  public static final String WINNER = "audio/match_result.wav";

  public abstract void addAudio(String name);
  public abstract void playAudio(String name);
  public abstract void playAudio(String name, double volume);
  public abstract void playAudioFromStart(String name, double volume);
  public abstract void changeVolume(String name, double volume);
  public abstract void stopAudio(String name);
}
