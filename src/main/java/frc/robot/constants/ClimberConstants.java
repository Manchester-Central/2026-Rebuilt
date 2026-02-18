// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;

import com.chaos131.can.CanConstants.CanBusName;
import com.chaos131.can.CanConstants.CanId;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public final class ClimberConstants {
    public static final CanBusName ClimberCanBus = CanBusName.CTRE;
    public static final CanId ClimberCanId = CanId.ID_20;

    public static final AnalogInput StringPotInput = new AnalogInput(0); // TODO: Double Check
    public static final Distance StringPotRange = Inches.of(10.0); // TODO: Double Check
    public static final Distance StringPotStartPoint = Inches.of(0); // TODO: Double Check

    public static final Distance MaxExtension = Inches.of(10); // TODO: Double Check
    public static final Distance ClimbExtension = Inches.of(6); // TODO: Double Check
    public static final Distance MinExtension = Inches.of(0); // TODO: Double Check

    public static final Mass ClimberMass = Kilogram.of(2); // TODO: Double Check
    public static final Distance DrivingDrumRadius = Meters.of(0.05); // TODO: Double Check

    public static double SensorToMechanismRatio = 1;

    public static final TalonFXConfiguration Config = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Amps.of(60)) // TODO: Double Check
            .withStatorCurrentLimit(Amps.of(60)) // TODO: Double Check
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
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKG(0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withGravityType(GravityTypeValue.Elevator_Static)
        );

    public static final DashboardNumber ManualSpeedMultiplier = new DashboardNumber("Climber/ManualSpeedMultiplier", 1);
}
