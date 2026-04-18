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

  private static final List<TableRow> LaunchRows = List.of(
    new TableRow(Meters.of(2.21), MetersPerSecond.of(15), 0.29), //15
    new TableRow(Meters.of(2.6), MetersPerSecond.of(17), 0.24),
    new TableRow(Meters.of(3.08), MetersPerSecond.of(20), 0.25), //20
    new TableRow(Meters.of(3.4), MetersPerSecond.of(22), 0.25), //21
    new TableRow(Meters.of(4), MetersPerSecond.of(30), 0.25) //28
    // new TableRow(Meters.of(3),  MetersPerSecond.of(38))
  );

  private static final List<TableRow> PassRows = List.of(
    new TableRow(Meters.of(2.21), MetersPerSecond.of(16), 0.29),
    new TableRow(Meters.of(2.6), MetersPerSecond.of(18), 0.24),
    new TableRow(Meters.of(3.08), MetersPerSecond.of(21.5), 0.25),
    new TableRow(Meters.of(3.4), MetersPerSecond.of(23), 0.25),
    new TableRow(Meters.of(4), MetersPerSecond.of(29), 0.25)
    // new TableRow(Meters.of(3),  MetersPerSecond.of(38))
  );

  private static final FlywheelTable m_launchInstance = new FlywheelTable(LaunchRows);
  private static final FlywheelTable m_passInstance = new FlywheelTable(PassRows);

  public static FlywheelTable getLaunchInstance() {
    return m_launchInstance;
  }

  public static FlywheelTable getPassInstance() {
    return m_passInstance;
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
