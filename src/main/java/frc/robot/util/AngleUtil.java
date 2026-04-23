// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class AngleUtil {
  public static boolean isNearWrapped(Angle angle1, Angle angle2, Angle tolerance) {
    double angleDegrees1 = angle1.in(Degrees);
    double angleDegrees2 = angle2.in(Degrees);
    double toleranceDegrees = tolerance.in(Degrees);

    if (angleDegrees1 < 0) {
      angleDegrees1 += 180;
    }

    if (angleDegrees2 < 0) {
      angleDegrees2 += 180;
    }

    return Math.abs(angleDegrees1 - angleDegrees2) < toleranceDegrees;
  }
}
