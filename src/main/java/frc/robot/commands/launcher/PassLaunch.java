// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.chaos131.poses.FieldPose;
import com.chaos131.util.DriveDirection;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.constants.LauncherConstants.HoodConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.TableRow;
import frc.robot.util.AngleUtil;

/**
 * Creates a launch command using the lookup table
 */
public class PassLaunch extends BaseLaunchCommand {
  private TableRow m_flywheelTableRow = new TableRow(Inches.of(0), MetersPerSecond.of(0), 0.0);

  public PassLaunch(Launcher launcher, Drive swerveDrive, Intake intake) {
    super(launcher, swerveDrive, intake);
  }

  @Override
  protected Optional<FieldPose> getTargetPose() {
    // The pose is not needed since we override isFacingTarget
    return Optional.empty();
  }

  @Override
  protected Optional<Distance> getTargetHeight() {
    // The height is not needed since we override isFacingTarget
    return Optional.empty();
  }

  @Override
  protected void preExecute() {
    m_flywheelTableRow = m_launcher.getPassLookupTableRow();
  }

  @Override
  protected void prepLauncher() {
    m_launcher.setFlywheelVelocity(m_flywheelTableRow.getLaunchSpeed(), true);
    m_launcher.setHoodAngle(HoodConstants.HoodMinAngle);
   }

  @Override
  protected void enableFeederForLauncher() {
    m_launcher.setFeederSpeed(FeederConstants.BottomFeederSpeed.get(), FeederConstants.TopFeederSpeed.get()); 
  }

  @Override
  protected boolean isFacingTarget(Angle tolerance) {
    Angle targetAngle =  DriveDirection.Towards.getAllianceAngle().getMeasure();
    Angle currentAngle = m_swerveDrive.getRotation().getMeasure();

    Logger.recordOutput("Launcher/TargetAngle", targetAngle.in(Degrees));
    Logger.recordOutput("Launcher/CurrentAngle", currentAngle.in(Degrees));
    Logger.recordOutput("Launcher/AngleDif", targetAngle.minus(currentAngle).in(Degrees));

    return AngleUtil.isNearWrapped(targetAngle, currentAngle, tolerance);
    // return targetAngle.isNear(currentAngle, tolerance);
  }

  @Override
  public Angle getIntakePivotAngle(){
    return getJostleAngle();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_launcher.setHoodAngle(HoodConstants.HoodMaxAngle);
  }
}
