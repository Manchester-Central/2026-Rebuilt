// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

import com.chaos131.can.CanConstants.CanBusName;
import com.chaos131.can.CanConstants.CanId;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public final class ClimberConstants {
    public static final CanBusName ClimberCanBus = CanBusName.RIO;
    public static final CanId ClimberCanId = CanId.ID_20;

    public static final double RotorToSensorRatio = 1; // TODO: Double Check
    public static final double SensorToMechanismRatio = 1; // TODO: Double Check

    public static final Distance MaxExtension = Inches.of(10); // TODO: Double Check
    public static final Distance MinExtension = Inches.of(0); // TODO: Double Check

    public static final InvertedValue MotorDirection = InvertedValue.Clockwise_Positive; // TODO: Double Check
    public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake; // TODO: Double Check

    public static final Current SupplyCurrentLimit = Amps.of(20); // TODO: Double Check
    public static final Current StatorCurrentLimit = Amps.of(20); // TODO: Double Check

    // Slot 0 Configs
    public static final double kP = 4; //TODO: CHECK THESE PLEASE
    public static final double kI = 0.1;
    public static final double kD = 1;
    public static final double kG = 0.1;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final DashboardNumber ManualSpeedMultiplier = new DashboardNumber("Climber/ManualSpeedMultiplier", 0.1, true, (x) -> {});
}
