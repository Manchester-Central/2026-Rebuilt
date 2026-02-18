// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.chaos131.can.CanConstants.CanBusName;
import com.chaos131.can.CanConstants.CanId;
import com.chaos131.poses.FieldPose2026;
import com.chaos131.util.DashboardNumber;
import com.chaos131.util.DashboardUnit;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public final class LauncherConstants {
  public static final CanBusName LauncherCanBus = CanBusName.CTRE;
  // public static final CanId TurretCanId = CanId.ID_43;

  public static Distance SimpleLauncherHeight = Inches.of(16); // TODO: Verify
  public static Angle SimpleLauncherAngle = Degrees.of(65);

  public static final DashboardNumber LauncherSpeed = new DashboardNumber("Launcher/ManualLaunchSpeed", 0.6); 

  public static final Distance LauncherToHubHeight = FieldDimensions.HubHeight.minus(SimpleLauncherHeight);

  public static final FieldPose2026 SafeLaunchePoint = new FieldPose2026(Alliance.Blue, "SafeLaunchPoint", new Pose2d(Inches.of(80), Inches.of(158.845), Rotation2d.kZero));
  public static final FieldPose2026 LeftPassPoint = new FieldPose2026(Alliance.Blue, "LeftPassPoint", new Pose2d(Inches.of(120), Inches.of(260), Rotation2d.kZero));
  public static final FieldPose2026 RightPassPoint = new FieldPose2026(Alliance.Blue, "RightPassPoint", new Pose2d(Inches.of(120), Inches.of(57.69), Rotation2d.kZero));
  public static final FieldPose2026[] PassPoints = new FieldPose2026[] {LeftPassPoint, RightPassPoint};

  public static final DashboardUnit<AngleUnit, Angle> AimYawTolerance = new DashboardUnit<>("Launcher/AimYawTolerance", Degrees.of(1));

  public static final DashboardUnit<TimeUnit, Time> AutoLaunchTime = new DashboardUnit<>("Launcher/AutoLaunchTime", Seconds.of(4.0));
  public static final class FlywheelConstants {
    public static final CanId LeftFlywheelCanId = CanId.ID_40;
    public static final CanId RightFlywheelCanId = CanId.ID_41;

    public static final Distance FlyWheelDiameter = Inches.of(6); //  TODO: Double Check
    public static final Mass FlywheelMass = Pounds.of(0.4);

    public static final LinearVelocity TargetVelocityTolerance = MetersPerSecond.of(0.2); // TODO: Double Check

    // Keep these separate to control them independently.
    // There's a 50-50 chance their directions are different!
    public static final TalonFXConfiguration LeftConfig = GenerateFlywheelConfig(InvertedValue.CounterClockwise_Positive);
    public static final TalonFXConfiguration RightConfig = GenerateFlywheelConfig(InvertedValue.Clockwise_Positive);

    private static TalonFXConfiguration GenerateFlywheelConfig(InvertedValue motorDirection) {
      return new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(motorDirection)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Amps.of(60))
            .withStatorCurrentLimit(Amps.of(60))
            .withSupplyCurrentLowerLimit(Amps.of(80))
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true)
        )
        .withFeedback(new FeedbackConfigs()
            .withRotorToSensorRatio(1) // TODO: Double Check
            .withSensorToMechanismRatio(1) // TODO: Double Check
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        )
        .withSlot0(new Slot0Configs() //TODO: CHECK THESE PLEASE
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKG(0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
        );
    }
  }

  public static final class IndexerConstants {
    public static final CanId IndexerCanId = CanId.ID_42;

    public static final DashboardNumber IndexerSpeed = new DashboardNumber("Launcher/Indexer/IndexerSpeed", 1);
    public static final DashboardNumber UnjamSpeed = new DashboardNumber("Launcher/Indexer/UnjamSpeed", -1);

    public static final TalonFXConfiguration Config = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.CounterClockwise_Positive)
          .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(Amps.of(60)) // TODO: Double Check
          .withStatorCurrentLimit(Amps.of(60)) // TODO: Double Check
          .withSupplyCurrentLowerLimit(Amps.of(80))
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimitEnable(true)
      );
  }
}
