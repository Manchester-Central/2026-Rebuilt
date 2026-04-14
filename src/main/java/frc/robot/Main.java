// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import java.util.Locale;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.MultiplayerSim.MatchAudio;
import frc.robot.subsystems.MultiplayerSim.MatchAudioOSX;
import frc.robot.subsystems.interfaces.AudioInterface;
import javafx.application.Platform;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    if (System.getProperty("os.name").toLowerCase(Locale.ENGLISH).contains("mac")) {
      System.out.println("*** Initializing OSX Audio");
      AudioInterface.instance = new MatchAudioOSX();
    } else {
      System.out.println("*** Initializing Windows Audio");
      AudioInterface.instance = new MatchAudio();
    }
    if (!System.getProperty("os.name").toLowerCase(Locale.ENGLISH).contains("mac")) {
      Platform.startup(() -> AudioInterface.getInstance());
    }
    AudioInterface.instance.loadAudioFiles();
    RobotBase.startRobot(Robot::new);
  }
}
