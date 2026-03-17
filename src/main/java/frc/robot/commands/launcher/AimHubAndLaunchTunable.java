// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import java.util.Optional;

import com.chaos131.poses.FieldPose;

import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.constants.LauncherConstants.FlywheelConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

/**
 * Creates a launch command using values from the dashboard
 */
public class AimHubAndLaunchTunable extends BaseLaunchCommand {

  public AimHubAndLaunchTunable(Launcher launcher, Drive swerveDrive, Intake intake) {
    super(launcher, swerveDrive, intake);
  }

  @Override
  protected Optional<FieldPose> getTargetPose() {
    return Optional.empty();
  }

  @Override
  protected Optional<Distance> getTargetHeight() {
    return Optional.empty();
  }

  @Override
  protected void prepLauncher() {
    m_launcher.setFlywheelVelocity(FlywheelConstants.TunableLaunchVelocity.get());
  }

  @Override
  protected void enableFeederForLauncher() {
    m_launcher.setFeederSpeed(FeederConstants.BottomFeederSpeed.get(), FeederConstants.TopFeederSpeed.get());
  }
}
