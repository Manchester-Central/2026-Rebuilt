// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.chaos131.poses.FieldPose2026;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    public static final int FlywheelCanId = 1;
    public static final int FlywheelCanId2 = 5;
    public static final String LauncherCanBus = RioCanBus;
    public static final int IndexerCanId = 2;
    public static final int TurretCanId = 3;

    public static Distance SimpleLauncherHeight = Inches.of(16); //TODO: Verify
    public static Angle SimpleLauncherAngle = Degrees.of(65);

    public static final Distance LauncherToHubHeight = FieldDimensions.HubHeight.minus(SimpleLauncherHeight);

    public static final FieldPose2026 LeftPassPoint = new FieldPose2026(Alliance.Blue, "LeftPassPoint", new Pose2d(Inches.of(120), Inches.of(260), Rotation2d.kZero));
    public static final FieldPose2026 RightPassPoint = new FieldPose2026(Alliance.Blue, "RightPassPoint", new Pose2d(Inches.of(120), Inches.of(57.69), Rotation2d.kZero));
  }

  public static final class FlywheelConstants {
    public static final Distance FlyWheelDiameter = Inches.of(6); //TODO: Double Check

    public static final double RotorToSensorRatio = 1; // TODO: Double Check
    public static final double SensorToMechanismRatio = 1; // TODO: Double Check

    public static final Distance MaxExtension = Inches.of(10); // TODO: Double Check
    public static final Distance MinExtension = Inches.of(0); // TODO: Double Check

    public static final InvertedValue[] MotorDirection = {InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive}; // TODO: Double Check
    public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake; // TODO: Double Check

    public static final Current SupplyCurrentLimit = Amps.of(20); // TODO: Double Check
    public static final Current StatorCurrentLimit = Amps.of(20); // TODO: Double Check

    // Slot 0 Configs
    public static final double kP = 0; //TODO: CHECK THESE PLEASE
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
  }
  
  public static final class ClimberConstants {
    public static final String ClimberCanBus = RioCanBus;
    public static final int ClimberCanId = 4;

    public static final double RotorToSensorRatio = 1; // TODO: Double Check
    public static final double SensorToMechanismRatio = 1; // TODO: Double Check

    public static final Distance MaxExtension = Inches.of(10); // TODO: Double Check
    public static final Distance MinExtension = Inches.of(0); // TODO: Double Check

    public static final InvertedValue MotorDirection = InvertedValue.Clockwise_Positive; // TODO: Double Check
    public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake; // TODO: Double Check

    public static final Current SupplyCurrentLimit = Amps.of(20); // TODO: Double Check
    public static final Current StatorCurrentLimit = Amps.of(20); // TODO: Double Check

    // Slot 0 Configs
    public static final double kP = 4; //TODO: CHECK THESE PLEASE
    public static final double kI = 0.1;
    public static final double kD = 1;
    public static final double kG = 0.1;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final DashboardNumber ManualSpeedMultiplier = new DashboardNumber("Climber/ManualSpeedMultiplier", 0.1, true, (x) -> {});
  }

  public static final class IntakeConstants {
    public static final int IntakeRollerCanId = 10;
    public static final String IntakeCanBus = RioCanBus;
    public static final int IntakePivotCanId = 11;
    public static final int IntakeKickerCanId = 12;
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
    public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.80665);
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

  public static final class FieldDimensions {
    public static Distance HubHeight = Inches.of(72);
  }
}
