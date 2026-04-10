package frc.robot.subsystems.MultiplayerSim;

import javafx.scene.media.Media;
import javafx.scene.media.MediaPlayer;
import java.io.File;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Filesystem;

public class MatchAudio {
  public static final String START = "audio/start.wav";
  public static final String RESUME = "audio/resume.wav";
  public static final String ENDGAME = "audio/warning.wav";
  public static final String WINNER = "audio/match_result.wav";

  private HashMap<String, MediaPlayer> audioMap = new HashMap<>();

  private static MatchAudio instance = new MatchAudio();
  public static MatchAudio getInstance() {
    return instance;
  }

  private MatchAudio() {
    addAudio(START);
    addAudio(ENDGAME);
    addAudio(RESUME);
    addAudio(WINNER);
  }

  /**
   * Adds an audio file to the audio map
   * @param name
   */
  public void addAudio(String name) {
    File deployDirectory = Filesystem.getDeployDirectory();
    var f = new File(deployDirectory, name);
    if (!f.exists()) {
      System.out.println("Audio file not found: " + name);
      return;
    }

    Media sound = new Media(f.toURI().toString());
    audioMap.put(name, new MediaPlayer(sound));
  }

  /**
   * Plays the audio file with the specified name
   * @param name
   */
  public void playAudio(String name) {
    if (audioMap.containsKey(name)) {
      if (audioMap.get(name).getStatus() != MediaPlayer.Status.PLAYING)
        audioMap.get(name).play();
    }
  }

  /**
   * Plays the audio file with the specified name
   * @param name
   */
  public void playAudio(String name, double volume) {
    if (audioMap.containsKey(name)) {
      if (audioMap.get(name).getStatus() != MediaPlayer.Status.PLAYING) {
        audioMap.get(name).setVolume(volume);
        audioMap.get(name).play();
      }
    }
  }

  public void playAudioFromStart(String name, double volume) {
    if (audioMap.containsKey(name)) {
      MediaPlayer player = audioMap.get(name);
      player.stop();
      player.setVolume(volume);
      player.play();
    }
  }

  /**
   * Changes the volume to the specified level, clamps between 0 and 1
   * @param name
   * @param volume
   */
  public void changeVolume(String name, double volume) {
    if (audioMap.containsKey(name)) {
      audioMap.get(name).setVolume(volume);
    }
  }

  /**
   * Stops the audio file with the specified name
   * @param name
   */
  public void stopAudio(String name) {
    if (audioMap.containsKey(name)) {
      audioMap.get(name).stop();
    }
  }
}