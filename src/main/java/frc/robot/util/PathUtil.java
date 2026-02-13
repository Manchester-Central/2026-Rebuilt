// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Set;

import com.chaos131.poses.FieldPose2026;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class PathUtil {
  public static Command driveToPoseCommand(Pose2d targetPose, Drive swerveDrive) {
    return new DeferredCommand(
      () -> AutoBuilder.pathfindToPose(targetPose, DriveConstants.pathConstraints), Set.of(swerveDrive));
  }

  public static Command driveToPoseCommand(FieldPose2026 targetPose, Drive swerveDrive) {
    return new DeferredCommand(
      () -> AutoBuilder.pathfindToPose(targetPose.getCurrentAlliancePose(), DriveConstants.pathConstraints), Set.of(swerveDrive));
  }
}
