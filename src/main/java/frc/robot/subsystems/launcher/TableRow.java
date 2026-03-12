// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.chaos131.tables.ITableRow;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/** Add your docs here. */
public class TableRow implements ITableRow<DistanceUnit, Distance> {

  private Distance m_distance;
  private LinearVelocity m_launchSpeed;
  private double m_feederSpeed;

  public TableRow(Distance distance, LinearVelocity launchSpeed, double feederSpeed) {
    m_distance = distance;
    m_launchSpeed = launchSpeed;
    m_feederSpeed = feederSpeed;
  }

  @Override
  public Distance getMeasure() {
    return m_distance;
  }

  public LinearVelocity getLaunchSpeed() {
    return m_launchSpeed;
  }

  public double getFeederSpeed() {
    return m_feederSpeed;
  }
}
