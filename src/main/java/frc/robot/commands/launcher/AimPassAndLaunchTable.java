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
import frc.robot.constants.LauncherConstants.HoodConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.TableRow;

/**
 * Creates a launch command that passes to a pass point using a lookup table
 */
public class AimPassAndLaunchTable extends BaseLaunchCommand {
  private TableRow m_flywheelTableRow = new TableRow(Inches.of(0), MetersPerSecond.of(0), 0.0);

  public AimPassAndLaunchTable(Launcher launcher, Drive swerveDrive, Intake intake) {
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
    m_flywheelTableRow = m_launcher.getPassLookupTableRow(); // TODO: might need to be a separate lookup table for hub launching
  }

  @Override
  protected void prepLauncher() {
    m_launcher.setFlywheelVelocity(m_flywheelTableRow.getLaunchSpeed());
    m_launcher.setHoodAngle(HoodConstants.HoodMinAngle);
  }

  @Override
  protected void enableFeederForLauncher() {
    m_launcher.setFeederSpeed(m_flywheelTableRow.getFeederSpeed());
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    m_launcher.setHoodAngle(HoodConstants.HoodMaxAngle);
  }
}
