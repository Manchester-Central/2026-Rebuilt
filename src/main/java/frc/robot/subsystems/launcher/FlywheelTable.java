// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

import com.chaos131.tables.LookupTable;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class FlywheelTable extends LookupTable<DistanceUnit, Distance, TableRow> {

  private static final List<TableRow> Rows = List.of(
    new TableRow(Meters.of(2.21), MetersPerSecond.of(15.5), 0.29),
    new TableRow(Meters.of(2.6), MetersPerSecond.of(17), 0.24),
    new TableRow(Meters.of(3.08), MetersPerSecond.of(20), 0.25),
    new TableRow(Meters.of(3.4), MetersPerSecond.of(21.4), 0.25),
    new TableRow(Meters.of(4), MetersPerSecond.of(28), 0.25)
    // new TableRow(Meters.of(3),  MetersPerSecond.of(38))
  );

  private static final FlywheelTable m_instance = new FlywheelTable(Rows);

  public static FlywheelTable getInstance() {
    return m_instance;
  }

  private FlywheelTable(List<TableRow> rows) {
    super(rows);
  }

  @Override
  public TableRow mergeRows(Distance targetMeasure, TableRow row1, TableRow row2) {
    var linearVelocity =
        interpolate(
            targetMeasure,
            row1,
            row2,
            (row) -> ((TableRow) row).getLaunchSpeed().in(MetersPerSecond));

    var feederSpeed = 
        interpolate(
            targetMeasure, 
            row1, 
            row2, 
            (row) -> ((TableRow) row).getFeederSpeed());
    return new TableRow(targetMeasure, MetersPerSecond.of(linearVelocity), feederSpeed);
  }
}
