// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.chaos131.can.CanConstants.CanBusName;
import com.chaos131.can.CanConstants.CanId;
import com.chaos131.ctre.CtreMotorSimValues;
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
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public final class LauncherConstants {
  public static final CanBusName LauncherCanBus = CanBusName.CTRE;
  // public static final CanId TurretCanId = CanId.ID_43;

  public static Distance LauncherHeight = Inches.of(16); // TODO: Verify
  public static Angle LauncherAngle = Degrees.of(65);

  public static final DashboardUnit<DistanceUnit, Distance> MaxLaunchHeight = new DashboardUnit<>("Launcher/MaxLaunchHeight", Feet.of(7));

  public static final DashboardNumber LauncherSpeed = new DashboardNumber("Launcher/ManualLaunchSpeed", 0.6); 
  public static final DashboardNumber UnjamSpeed = new DashboardNumber("Launcher/UnjamSpeed", -0.3); 

  public static final Distance LauncherToHubHeight = FieldDimensions.HubHeight.minus(LauncherHeight);

  public static final FieldPose2026 SafeLaunchPoint = new FieldPose2026(Alliance.Blue, "SafeLaunchPoint", new Pose2d(Inches.of(80), Inches.of(158.845), Rotation2d.kZero));
  public static final FieldPose2026 LeftPassPoint = new FieldPose2026(Alliance.Blue, "LeftPassPoint", new Pose2d(Inches.of(120), Inches.of(260), Rotation2d.kZero));
  public static final FieldPose2026 RightPassPoint = new FieldPose2026(Alliance.Blue, "RightPassPoint", new Pose2d(Inches.of(120), Inches.of(57.69), Rotation2d.kZero));
  public static final FieldPose2026[] PassPoints = new FieldPose2026[] {LeftPassPoint, RightPassPoint};

  public static final DashboardUnit<AngleUnit, Angle> AimYawTolerance = new DashboardUnit<>("Launcher/AimYawTolerance", Degrees.of(1)); // tested with 3

  public static final DashboardUnit<TimeUnit, Time> AutoLaunchTime = new DashboardUnit<>("Launcher/AutoLaunchTime", Seconds.of(4.0));
  public static final class FlywheelConstants {
    public static final CanId LeftFlywheelCanId = CanId.ID_40;
    public static final CanId RightFlywheelCanId = CanId.ID_41;

    public static final Distance FlyWheelDiameter = Inches.of(6); //  TODO: Double Check
    public static final Mass FlywheelMass = Pounds.of(0.4);

    public static final DashboardUnit<LinearVelocityUnit, LinearVelocity> TargetVelocityTolerance = new DashboardUnit<>("Launcher/TargetVelocityTolerance", MetersPerSecond.of(0.4)); // TODO: Tested with 2

    public static final double SensorToMechanismRatio = 1; // TODO check or change

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
            .withSensorToMechanismRatio(SensorToMechanismRatio) // TODO: Double Check
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        )
        .withSlot0(new Slot0Configs() //TODO: CHECK THESE PLEASE
            .withKP(0.15)
            .withKI(0)
            .withKD(0.02)
            .withKG(0)
            .withKS(0.1)
            .withKV(0.12)
            .withKA(0)
        );
    }

    // Sim values
    public static final double MOI = SingleJointedArmSim.estimateMOI(FlyWheelDiameter.in(Meters), FlywheelMass.in(Kilograms));
    public static final DCMotor DcMotor = DCMotor.getKrakenX60(2);
    public static final DCMotorSim DcMotorSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DcMotor, MOI, SensorToMechanismRatio),
      DcMotor);
    public static final CtreMotorSimValues SimValues = new CtreMotorSimValues(
      DcMotorSim,
      SensorToMechanismRatio,
      true,
      CtreMotorSimValues.chassisReferenceFromInvertedValue(LeftConfig.MotorOutput.Inverted),
      MotorType.KrakenX60);
  }

  public static final class HoodConstants {
    public static final CanId HoodCanId = CanId.ID_44;
    public static final int SensorIndex = 2;
    public static final DashboardNumber HoodSpeed = new DashboardNumber("Launcher/Hood/HoodSpeed", 0.2);
    public static final DashboardNumber NotReachedMaxSpeed = new DashboardNumber("Launcher/Hood/NotReachedMaxSpeed", 0.2);
    public static final Distance HoodRadius = Inches.of(9);
    public static final Mass HoodMass = Kilogram.of(2.26796); 
    public static final Angle HoodMinAngle = Degrees.of(40);
    public static final Angle HoodMaxAngle = Degrees.of(85); 

    public static final double SensorToMechanismRatio = 19; // TODO check or change
    public static final TalonFXConfiguration HoodConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) // TODO: check
            .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Amps.of(20)) // TODO: Double CHek
            .withStatorCurrentLimit(Amps.of(20)) // TODO: Double Check
            .withSupplyCurrentLowerLimit(Amps.of(40))
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true))
        .withFeedback(new FeedbackConfigs()
            .withRotorToSensorRatio(1) // TODO: Double Check
            .withSensorToMechanismRatio(SensorToMechanismRatio) // TODO: Double Check
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
        .withSlot0(new Slot0Configs() // TODO: CHECK THESE PLEASE
            .withKP(0) // 20.0 tested in sim
            .withKI(0)
            .withKD(0)
            .withKG(0)
            .withKS(0)
            .withKV(0)
            .withKA(0));

    // Sim values
    public static final double MOI = SingleJointedArmSim.estimateMOI(HoodRadius.in(Meters), HoodMass.in(Kilograms));
    public static final DCMotor DcMotor = DCMotor.getKrakenX44(1);
    public static final DCMotorSim DcMotorSim = new DCMotorSim(
      LinearSystemId.createSingleJointedArmSystem(DcMotor, MOI, SensorToMechanismRatio),
      DcMotor);
    public static final CtreMotorSimValues SimValues = new CtreMotorSimValues(
      DcMotorSim,
      SensorToMechanismRatio,
      true,
      CtreMotorSimValues.chassisReferenceFromInvertedValue(HoodConfig.MotorOutput.Inverted),
      MotorType.KrakenX44);
  }

  public static final class FeederConstants {
    public static final CanId TopFeederCanId = CanId.ID_42;
    public static final CanId BottomFeederCanId = CanId.ID_43;
    public static final int SensorIndex = 1;

    public static final DashboardNumber UnjamSpeed = new DashboardNumber("Launcher/Feeder/UnjamSpeed", -1.0);
    public static final DashboardNumber FeederSpeed = new DashboardNumber("Launcher/Feeder/FeederSpeed", 1.0);
    public static final DashboardNumber PassiveFeederSpeed = new DashboardNumber("Launcher/Feeder/PassiveFeederSpeed", 0.2);

    public static final TalonFXConfiguration TopConfig = new TalonFXConfiguration()
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
     public static final TalonFXConfiguration BottomConfig = new TalonFXConfiguration()
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
