// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;

public final class QuestConstants {
  public static final Distance RobotToQuestXInches = Inches.of(8.25); // TODO: Get Value from CAD
  public static final Distance RobotToQuestYInches = Inches.of(-12.75); // TODO: Get Value from CAD
  public static final Distance RobotToQuestZInches = Inches.of(12); // TODO: Get Value from CAD
  public static final Rotation3d RobotToQuestRotation = new Rotation3d(Degrees.of(0),Degrees.of(0),Degrees.of(-90)); // TODO: Get Value from CAD
}
