// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public final class RobotDimensions {
  // Thickness of just the bumpers
  public static Distance BumperThickness = Inches.of(1.5);
  // From left to right
  public static Distance FrameWidth = Inches.of(28);
  // From front to back
  public static Distance FrameLength = Inches.of(25.5);
  // BumperWidth
  // BumperLength

  // Distance of the intake's range
  public static Distance IntakeRange = Inches.of(8);

  // Highly tunable angle that reflects how the ball releases from the hood
  public static Angle FixedHoodAngle = Degrees.of(60);
  // Intake Axle
  public static Translation3d IntakeOffset = new Translation3d(Meters.of(-0.275449), Meters.of(0), Meters.of(0.222401));

  public static Distance LauncherWidth = Meters.of(0.6);
}
