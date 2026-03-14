// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;

import com.chaos131.can.CanConstants.CanBusName;
import com.chaos131.can.CanConstants.CanId;
import com.chaos131.ctre.CtreMotorSimValues;
import com.chaos131.util.DashboardNumber;
import com.chaos131.util.DashboardUnit;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.encoder.config.DetachedEncoderConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public final class IntakeConstants {
  public static final CanBusName CanBus = CanBusName.CTRE;

  // Intake Dimensions
  public static final Distance IntakeLength = Inches.of(6); // TODO: Double Check
  public static final Mass IntakeMass = Pounds.of(5); // TODO: Double Check

  // Manual Multipliers
  public static final DashboardNumber ManualPivotSpeedMultiplier = new DashboardNumber("Intake/ManualPivotSpeedMultiplier", 0.4);

  // Speeds
  public static final DashboardNumber IntakeRollerSpeed = new DashboardNumber("Intake/IntakeRollerSpeed", 0.85);
  public static final DashboardNumber OuttakeRollerSpeed = new DashboardNumber("Intake/OuttakeRollerSpeed", -0.4);

  public static final class RollerConstants {
    public static final CanId RollerCanId = CanId.ID_30;

    public static final TalonFXConfiguration Config = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withInverted(InvertedValue.Clockwise_Positive)
          .withNeutralMode(NeutralModeValue.Coast)
      )
      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimit(Amps.of(80)) // TODO: Double Check
          .withStatorCurrentLimit(Amps.of(80)) // TODO: Double Check
          .withSupplyCurrentLowerLimit(Amps.of(80))
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimitEnable(true)
      );
  }

  public static final class PivotConstants {
    public static final CanId PivotCanId = CanId.ID_31;
    public static final CanId PivotCanCoderId = CanId.ID_32;

    public static double SensorToMechanismRatio = 42.603; //TODO: tune 

    public static final TalonFXConfiguration TalonConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Amps.of(50)) // TODO: Double Check
            .withStatorCurrentLimit(Amps.of(50)) // TODO: Double Check
            .withSupplyCurrentLowerLimit(Amps.of(60))
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true)
        )
        .withFeedback(new FeedbackConfigs()
            .withRotorToSensorRatio(1) // TODO: Double Check
            .withSensorToMechanismRatio(SensorToMechanismRatio) // TODO: Double Check
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        )
        .withSlot0(new Slot0Configs() //TODO: CHECK THESE PLEASE
            .withKP(60)
            .withKI(0)
            .withKD(0)
            .withKG(0.39)
            .withKS(0.29)
            .withKV(0)
            .withKA(0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        );

    public static final DetachedEncoderConfig pivotEncoderConfig = new DetachedEncoderConfig()
        .inverted(true)
        .dutyCycleOffset(Degrees.of(120).in(Rotations));

    // Pivot Max / Min
    public static final Angle MaxAngle = Degrees.of(177); // TODO: Double Check
    public static final Angle MinAngle = Degrees.of(49); // TODO: Double Check

    // Target Angles / Speeds
    public static final DashboardUnit<AngleUnit,Angle> DeployAngle = new DashboardUnit<>("Intake/DeployAngle", Degrees.of(172)); // TODO: Double Check
    public static final DashboardUnit<AngleUnit,Angle> RetractAngle = new DashboardUnit<>("Intake/RetractAngle", Degrees.of(110)); // TODO: Double Check

    // Sim values
    public static final double MOI = SingleJointedArmSim.estimateMOI(IntakeLength.in(Meters), IntakeMass.in(Kilograms));
    public static final DCMotor DcMotor = DCMotor.getKrakenX60(1);
    public static final DCMotorSim DcMotorSim = new DCMotorSim(
      LinearSystemId.createSingleJointedArmSystem(DcMotor, MOI, SensorToMechanismRatio),
      DcMotor);
    public static final CtreMotorSimValues SimValues = new CtreMotorSimValues(
      DcMotorSim,
      SensorToMechanismRatio,
      true,
      CtreMotorSimValues.chassisReferenceFromInvertedValue(TalonConfig.MotorOutput.Inverted),
      MotorType.KrakenX60);
  }
}
