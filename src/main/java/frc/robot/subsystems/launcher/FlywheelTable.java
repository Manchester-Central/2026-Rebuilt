// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.chaos131.tables.LookupTable;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class FlywheelTable extends LookupTable<DistanceUnit, Distance, TableRow> {
  @Override
  public TableRow mergeRows(Distance targetMeasure, TableRow row1, TableRow row2) {
    var linearVelocity =
        interpolate(
            targetMeasure,
            row1,
            row2,
            (row) -> ((TableRow) row).getLaunchSpeed().in(MetersPerSecond));
    return new TableRow(targetMeasure, MetersPerSecond.of(linearVelocity));
  }

  private static LookupTable<DistanceUnit, Distance, TableRow> m_instance = new FlywheelTable()
    .addRow(new TableRow(Meters.of(1), MetersPerSecond.of(24)))
    .addRow(new TableRow(Meters.of(1), MetersPerSecond.of(32)))
    .addRow(new TableRow(Meters.of(3),  MetersPerSecond.of(38)));

  public static LookupTable<DistanceUnit, Distance, TableRow> getInstance() {
    return m_instance;
  }
}
