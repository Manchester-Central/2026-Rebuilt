// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;

import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interfaces.AbstractDrive;
import frc.robot.subsystems.launcher.Launcher;

/**
 * Creates a launch command that passes to a pass point using physics WITHOUT a moving hood
 */
public class AimPassAndLaunchSetAngle extends BaseLaunchCommand {

  public AimPassAndLaunchSetAngle(Launcher launcher, AbstractDrive swerveDrive, Intake intake) {
    super(launcher, swerveDrive, intake);
  }

  @Override
  protected Optional<FieldPose> getTargetPose() {
    return Optional.of(FieldPose2026.getClosestPose(m_swerveDrive.getPose(), LauncherConstants.PassPoints));
  }

  @Override
  protected Optional<Distance> getTargetHeight() {
    return Optional.of(Inches.of(0));
  }

  @Override
  protected void prepLauncher() {
    m_launcher.setFlywheelVelocity(m_launcher.getPassVelocitySetAngle(m_swerveDrive.getPose()));
  }

  @Override
  protected void enableFeederForLauncher() {
    m_launcher.setFeederSpeed(FeederConstants.FeederSpeed.get());
  }
}
