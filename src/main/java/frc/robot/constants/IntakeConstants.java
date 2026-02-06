// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;

import com.chaos131.can.CanConstants.CanBusName;
import com.chaos131.can.CanConstants.CanId;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/** Add your docs here. */
public final class IntakeConstants {
  public static final CanBusName CanBus = CanBusName.RIO;
  // public static final int IntakeKickerCanId = 13;

  // Intake Dimensions
  public static final Distance IntakeLength = Inches.of(6);
  public static final Mass IntakeMass = Pounds.of(5);

  
  // Manual Multipliers
  public static final DashboardNumber ManualPivotSpeedMultiplier = new DashboardNumber("Intake/ManualPivotSpeedMultiplier", 0.1, true, (x) -> {});

  public static final class RollerConstants {
    public static final CanId RollerCanId = CanId.ID_30;

    // Roller config
    public static final InvertedValue MotorDirection = InvertedValue.Clockwise_Positive; // TODO: Double Check
    public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake; // TODO: Double Check

    public static final Current SupplyCurrentLimit = Amps.of(20); // TODO: Double Check
    public static final Current StatorCurrentLimit = Amps.of(20); // TODO: Double Check
  }


  // Kicker config
  // public static final InvertedValue KickerMotorDirection = InvertedValue.Clockwise_Positive; // TODO: Double Check
  // public static final NeutralModeValue KickerNeutralMode = NeutralModeValue.Brake; // TODO: Double Check

  // public static final Current KickerSupplyCurrentLimit = Amps.of(20); // TODO: Double Check
  // public static final Current KickerStatorCurrentLimit = Amps.of(20); // TODO: Double Check

  public static final class PivotConstants {
    public static final CanId PivotCanId = CanId.ID_31;
    public static final CanId PivotCanCoderId = CanId.ID_32;

    // Pivot Config
    public static final double RotorToSensorRatio = 1; // TODO: Double Check
    public static final double SensorToMechanismRatio = 1; // TODO: Double Check
    public static final InvertedValue MotorDirection = InvertedValue.Clockwise_Positive; // TODO: Double Check
    public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake; // TODO: Double Check

    public static final Current SupplyCurrentLimit = Amps.of(20); // TODO: Double Check
    public static final Current StatorCurrentLimit = Amps.of(20); // TODO: Double Check

    // Pivot CanCoder Config
    public static final SensorDirectionValue CanCoderDirection = SensorDirectionValue.Clockwise_Positive; // TODO: Double Check
    public static final Angle CanCoderDiscontinuityPoint = Degrees.of(270); // TODO: Double Check
    public static final Angle CanCoderOffset = Rotations.of(0);
    

    // Pivot Slot 0 Configs
    public static final double kP = 0; //TODO: CHECK THESE PLEASE
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    // Pivot Max / Min
    public static final Angle MaxAngle = Degrees.of(190); // TODO: Double Check
    public static final Angle MinAngle = Degrees.of(95); // TODO: Double Check

    // Target Angles / Speeds
    public static final Angle RetractAngle = Degrees.of(90); // TODO: Double Check
    public static final Angle DeployAngle = Degrees.of(180); // TODO: Double Check
  }
}
