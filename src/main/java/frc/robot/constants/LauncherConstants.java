// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.chaos131.can.CanConstants.CanBusName;
import com.chaos131.can.CanConstants.CanId;
import com.chaos131.poses.FieldPose2026;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public final class LauncherConstants {
  public static final CanBusName LauncherCanBus = CanBusName.RIO;
  public static final CanId LeftFlywheelCanId = CanId.ID_40;
  public static final CanId RightFlywheelCanId = CanId.ID_41;
  public static final CanId IndexerCanId = CanId.ID_42;
  public static final CanId TurretCanId = CanId.ID_43;

  public static Distance SimpleLauncherHeight = Inches.of(16); //TODO: Verify
  public static Angle SimpleLauncherAngle = Degrees.of(65);

  public static final Distance LauncherToHubHeight = FieldDimensions.HubHeight.minus(SimpleLauncherHeight);

  public static final FieldPose2026 LeftPassPoint = new FieldPose2026(Alliance.Blue, "LeftPassPoint", new Pose2d(Inches.of(120), Inches.of(260), Rotation2d.kZero));
  public static final FieldPose2026 RightPassPoint = new FieldPose2026(Alliance.Blue, "RightPassPoint", new Pose2d(Inches.of(120), Inches.of(57.69), Rotation2d.kZero));

  public static final class FlywheelConstants {
    public static final Distance FlyWheelDiameter = Inches.of(6); //TODO: Double Check

    public static final double RotorToSensorRatio = 1; // TODO: Double Check
    public static final double SensorToMechanismRatio = 1; // TODO: Double Check

    public static final Distance MaxExtension = Inches.of(10); // TODO: Double Check
    public static final Distance MinExtension = Inches.of(0); // TODO: Double Check

    public static final InvertedValue LeftMotorDirection = InvertedValue.Clockwise_Positive;
    public static final InvertedValue RightMotorDirection = InvertedValue.Clockwise_Positive; // TODO: Double Check
    public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake; // TODO: Double Check

    public static final Current SupplyCurrentLimit = Amps.of(20); // TODO: Double Check
    public static final Current StatorCurrentLimit = Amps.of(20); // TODO: Double Check

    // Slot 0 Configs
    public static final double kP = 0; //TODO: CHECK THESE PLEASE
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
  }
}
