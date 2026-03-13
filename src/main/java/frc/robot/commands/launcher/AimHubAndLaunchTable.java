// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.FieldDimensions;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.subsystems.interfaces.IDrive;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.ILauncher;
import frc.robot.subsystems.launcher.TableRow;

/**
 * Creates a launch command using the lookup table
 */
public class AimHubAndLaunchTable extends BaseLaunchCommand {
  private TableRow m_flywheelTableRow = new TableRow(Inches.of(0), MetersPerSecond.of(0), 0.0);

  private Timer m_intakeTimer = new Timer();

  public AimHubAndLaunchTable(ILauncher launcher, IDrive swerveDrive, IIntake intake) {
    super(launcher, swerveDrive, intake);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_intakeTimer.restart();
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
  protected void preExecute() {
    m_flywheelTableRow = m_launcher.getLookupTableRow();
  }

  @Override
  protected void prepLauncher() {
    m_launcher.setFlywheelVelocity(m_flywheelTableRow.getLaunchSpeed());
  }

  @Override
  protected void enableFeederForLauncher() {
    m_launcher.setFeederSpeed(FeederConstants.BottomFeederSpeed.get(), FeederConstants.TopFeederSpeed.get()); 
  }

  @Override
  public Angle getIntakePivotAngle(){
    Angle pivotAngle = super.getIntakePivotAngle();
    return pivotAngle.minus(Degree.of(m_intakeTimer.get()));
  }

}
