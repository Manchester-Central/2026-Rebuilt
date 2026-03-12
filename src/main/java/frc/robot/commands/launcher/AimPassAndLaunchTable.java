// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;

import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.interfaces.IDrive;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.ILauncher;
import frc.robot.subsystems.launcher.TableRow;

/**
 * Creates a launch command that passes to a pass point using a lookup table
 */
public class AimPassAndLaunchTable extends BaseLaunchCommand {
  private TableRow m_flywheelTableRow = new TableRow(Inches.of(0), MetersPerSecond.of(0), 0.0);

  public AimPassAndLaunchTable(ILauncher launcher, IDrive swerveDrive, IIntake intake) {
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
  protected void preExecute() {
    m_flywheelTableRow = m_launcher.getLookupTableRow(); // TODO: might need to be a separate lookup table for hub launching
  }

  @Override
  protected void prepLauncher() {
    m_launcher.setFlywheelVelocity(m_flywheelTableRow.getLaunchSpeed());
  }

  @Override
  protected void enableFeederForLauncher() {
    m_launcher.setFeederSpeed(m_flywheelTableRow.getFeederSpeed());
  }
}
