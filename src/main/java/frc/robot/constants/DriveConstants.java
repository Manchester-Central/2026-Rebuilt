// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.generated.TunerConstants;

/** Constants for the drive.
 * @see TunerConstants TunerConstants for variables controlled by the CTRE generator, like CAN IDs, module PID values, etc.
 */
public final class DriveConstants {

  // PathPlanner config constants - these values should match the settings in the PathPlanner UI. 
  public static final Mass RobotMass = Kilograms.of(74.088); // TODO: confirm
  public static final MomentOfInertia RobotMoi = KilogramSquareMeters.of(6.883); // TODO: confirm
  public static final double WheelCof = 1.2; // TODO: confirm
  public static final PIDConstants TranslationalControlPIDConstants = new PIDConstants(5.0, 0.0, 0.0); // TODO: confirm
  public static final PIDConstants RotationalControlPIDConstants = new PIDConstants(5.0, 0.0, 0.0); // TODO: confirm

  // Path Constraints
  public static final LinearVelocity PathVelocityConstraint = MetersPerSecond.of(3.0); // TODO: update values
  public static final LinearAcceleration PathAccelerationConstraint = MetersPerSecondPerSecond.of(4.0); // TODO: update values
  public static final AngularVelocity PathAngularVelocity = RotationsPerSecond.of(1.5); // TODO: update values
  public static final AngularAcceleration PathAngularAcceleration = RotationsPerSecondPerSecond.of(2); // TODO: update values

  public static final PathConstraints pathConstraints = new PathConstraints(PathVelocityConstraint, PathAccelerationConstraint, PathAngularVelocity, PathAngularAcceleration);

}
