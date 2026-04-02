// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.FieldDimensions;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.LauncherConstants;

/** Add your docs here. */
public class LauncherUtil {
  /**
   * Determines launch velocity needed to hit a certain target from any position asumming a constant launch angle.
   * 
   * @param currentPose The pose of the robot when launching.
   * @param targetPose The pose of the target to launch.
   * @param targetHeight The height to target at the specified pose.
   * @return The linear velocity needed to launch at the target.
   */
  private static LinearVelocity getVelocityForTargetSetAngle(Pose2d currentPose, Pose2d targetPose, Distance targetHeight) {
    double groundDisplacementInches = FieldPose.getDistanceFromLocations(currentPose, targetPose).in(Inches);
    double thetaRadians = LauncherConstants.LauncherAngle.in(Radians);
    double deltaHeightInches = targetHeight.minus(LauncherConstants.LauncherHeight).in(Inches);
    
    double numerator = GeneralConstants.gravity.in(InchesPerSecondPerSecond) * Math.pow(groundDisplacementInches, 2);
    double denomenator = 2 * Math.pow(Math.cos(thetaRadians), 2) * (deltaHeightInches - groundDisplacementInches * Math.tan(thetaRadians));

    double velocity = Math.sqrt(numerator / denomenator);
    return InchesPerSecond.of(velocity);
  }

  /**
   * Determines launch velocity needed to score in the hub from any position asumming a constant launch angle.
   * 
   * @param currentPose The pose of the robot when launching.
   * @return The linear velocity needed to launch in the hub.
   */
  public static LinearVelocity getScoringVelocitySetAngle(Pose2d currentPose) {
    return getVelocityForTargetSetAngle(currentPose, FieldPose2026.HubCenter.getCurrentAlliancePose(), FieldDimensions.HubHeight);
  }

  /**
   * Determines launch velocity needed to pass to a pass point from any position asumming a constant launch angle.
   * 
   * @param currentPose The pose of the robot when launching.
   * @return The linear velocity needed to launch at the closest pass point.
   */
  public static LinearVelocity getPassVelocitySetAngle(Pose2d currentPose) {
    FieldPose closestPose = FieldPose.getClosestPose(currentPose, LauncherConstants.PassPoints);
    return getVelocityForTargetSetAngle(currentPose, closestPose.getCurrentAlliancePose(), Inches.of(0));
  }
}
