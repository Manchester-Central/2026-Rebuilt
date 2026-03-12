// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import java.util.Optional;

import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;

import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.FieldDimensions;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.subsystems.interfaces.IDrive;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.ILauncher;

/**
 * Creates a launch command using physics WITHOUT a moving hood
 */
public class AimHubAndLaunchSetAngle extends BaseLaunchCommand {

  public AimHubAndLaunchSetAngle(ILauncher launcher, IDrive swerveDrive, IIntake intake) {
    super(launcher, swerveDrive, intake);
  }

  @Override
  protected Optional<FieldPose> getTargetPose() {
    return Optional.of(FieldPose2026.HubCenter);
  }

  @Override
  protected Optional<Distance> getTargetHeight() {
    return Optional.of(FieldDimensions.HubHeight);
  }

  @Override
  protected void prepLauncher() {
    m_launcher.setFlywheelVelocity(m_launcher.getScoringVelocitySetAngle(m_swerveDrive.getPose()));
  }

  @Override
  protected void enableFeederForLauncher() {
    m_launcher.setFeederSpeed(FeederConstants.FeederSpeed.get());
  }

}
