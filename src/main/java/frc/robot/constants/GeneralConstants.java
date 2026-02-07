// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.RobotBase;

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

/** Add your docs here. */
public final class GeneralConstants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final double SlowModeMultiplier = 0.5;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // General Game Info Here!
  public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(-9.80665);
}
