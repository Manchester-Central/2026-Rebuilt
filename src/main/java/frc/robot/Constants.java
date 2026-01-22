// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final String RioCanBus = "rio";

  public static final class LauncherConstants {
    public static final int LauncherCanId = 1;
    public static final String LauncherCanBus = RioCanBus;
  }
  
  public static final class IntakeConstants {
    public static final int IntakeCanId = 10;
    public static final String IntakeCanBus = RioCanBus;
  }

  public static final class QuestConstants {
    public static final Distance RobotToQuestXInches = Inches.of(5);
    public static final Distance RobotToQuestYInches = Inches.of(5);
    public static final Distance RobotToQuestZInches = Inches.of(5);
    public static final Rotation3d RobotToQuestRotation = Rotation3d.kZero;
  }

  public static final class LimelightConstants {
    public static final Pose3d RobotToLimelight = new Pose3d();
  }

  /*
    ⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⡿⢿⡿⢉⣹⡟⣿⡿⣤⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀
    ⠀⠀⠀⠀⠀⠀⢀⣠⣶⠿⢋⡐⣾⣷⡟⣫⣼⣷⠭⠯⣝⣞⡻⢷⣤⣀⠀⠀⠀⠀
    ⠀⠀⠀⠀⣠⣼⣿⠟⠃⡜⣠⣿⢿⣣⣼⣿⡿⠛⠀⠀⠀⠀⠛⢧⣜⠿⣧⡀⠀⠀
    ⠀⢀⣴⡿⠟⢃⠌⡰⡑⣾⣿⣟⣯⣿⡿⠉⠀⠀⠀⠀⠀⠀⠀⠀⠈⢻⣜⣻⣤⠀
    ⣴⣿⠏⢡⠘⠌⠦⢱⣹⣿⠷⢋⣤⡿⢀⠠⠀⠀⠀⠀⠀⢀⠠⢈⠐⡀⠘⣧⢻⢧
    ⣿⠃⣘⠂⣍⠚⣬⣵⡟⢡⣊⣾⢫⣿⣿⣦⠀⠀⠠⢠⣾⣿⣷⠀⠂⠌⢡⠈⢧⣳
    ⣟⠠⣁⠮⣔⡹⣞⠧⣜⣶⡟⠣⢸⣿⣾⡟⠀⠠⠀⠘⣿⣾⣽⠃⠌⡐⢂⡉⢆⣧
    ⣿⣦⣝⣴⣽⣷⣿⣶⠿⣋⠕⢢⠈⠛⡛⢁⠀⢂⠐⠠⠙⠟⣉⠐⡠⠃⡌⠱⣌⣽
    ⠙⠛⠚⠓⢻⣿⡳⡜⡒⢄⠊⡄⢃⠒⢄⠂⣁⠂⠌⡐⣁⠢⢀⠣⡐⠡⢌⡓⡴⣺
    ⠀⠀⠀⠀⠐⣿⣷⡹⡜⡌⠦⡑⡌⠸⣀⠣⡐⢌⠢⡑⢄⢊⠴⢡⠘⡥⢊⡴⣳⢝
    ⠀⠀⠀⠀⠀⠘⡿⣷⡹⣌⢳⡐⠻⠷⡾⢷⡷⣾⢷⡾⡾⠾⠾⣃⠞⣰⣋⢶⠍⡼
    ⠀⠀⠀⠀⠀⠀⠘⢻⣿⡜⣧⠙⣧⠓⡜⢢⠒⡔⢢⠒⣥⠋⣶⢱⢺⣥⠛⣮⡞⠀
    ⠀⠀⠀⠀⠀⠀⠀⠈⠻⣿⣭⢷⣎⡳⣵⣊⡞⣱⢎⠶⣱⣚⢦⣛⣶⣏⣷⠋⠀⠀
    ⠀⠀⠀⠀⠀⠀⠀⠀⠈⢛⢿⡾⣷⢧⡷⣹⣳⢮⡟⡵⢯⣾⣟⣿⠋⠀⠀⠀⠀
   */
  public static final class GeneralConstants {
    // General Game Info Here!
  }

  public static final class RobotDimensions {
    // Thickness of just the bumpers
    public static Distance BumperThickness = Inches.of(1.5);
    // From left to right
    public static Distance FrameWidth = Inches.of(30);
    // From front to back
    public static Distance FrameLength = Inches.of(30);
    // BumperWidth
    // BumperLength
  }
}
