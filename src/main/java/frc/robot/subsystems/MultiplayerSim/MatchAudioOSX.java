package frc.robot.subsystems.MultiplayerSim;

import javafx.scene.media.MediaPlayer;
import java.util.HashMap;

import frc.robot.subsystems.interfaces.AudioInterface;

public class MatchAudioOSX extends AudioInterface {
  public static final String START = "audio/start.wav";
  public static final String RESUME = "audio/resume.wav";
  public static final String ENDGAME = "audio/warning.wav";
  public static final String WINNER = "audio/match_result.wav";

  @SuppressWarnings("unused")
  private HashMap<String, MediaPlayer> audioMap = new HashMap<>();

  public MatchAudioOSX() {}

  @Override
  public void loadAudioFiles() {
    addAudio(START);
    addAudio(ENDGAME);
    addAudio(RESUME);
    addAudio(WINNER);
  }

  /**
   * Adds an audio file to the audio map
   * @param name
   */
  @Override
  public void addAudio(String name) {}

  /**
   * Plays the audio file with the specified name
   * @param name
   */
  @Override
  public void playAudio(String name) {}

  /**
   * Plays the audio file with the specified name
   * @param name
   */
  @Override
  public void playAudio(String name, double volume) {}

  @Override
  public void playAudioFromStart(String name, double volume) {}

  /**
   * Changes the volume to the specified level, clamps between 0 and 1
   * @param name
   * @param volume
   */
  @Override
  public void changeVolume(String name, double volume) {}

  /**
   * Stops the audio file with the specified name
   * @param name
   */
  @Override
  public void stopAudio(String name) {}
}