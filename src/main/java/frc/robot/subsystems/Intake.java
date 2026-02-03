// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.chaos131.util.ChaosTalonFx;
import com.chaos131.util.ChaosTalonFxTuner;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.interfaces.IIntake;

public class Intake extends SubsystemBase implements IIntake {
  private ChaosTalonFx m_intakeRollerMotor = new ChaosTalonFx(IntakeConstants.IntakeRollerCanId, IntakeConstants.IntakeCanBus);
  private ChaosTalonFx m_intakeKickerMotor = new ChaosTalonFx(IntakeConstants.IntakeKickerCanId, IntakeConstants.IntakeCanBus);
  private ChaosTalonFx m_intakePivotMotor = new ChaosTalonFx(IntakeConstants.IntakePivotCanId, IntakeConstants.IntakeCanBus);
  private ChaosTalonFxTuner m_intakePivotTuner = new ChaosTalonFxTuner("PivotTuner", m_intakePivotMotor);

  private DashboardNumber m_kp = m_intakePivotTuner.tunable("kP", IntakeConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
  private DashboardNumber m_ki = m_intakePivotTuner.tunable("kI", IntakeConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
  private DashboardNumber m_kd = m_intakePivotTuner.tunable("kD", IntakeConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
  private DashboardNumber m_kg = m_intakePivotTuner.tunable("kG", IntakeConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
  private DashboardNumber m_ks = m_intakePivotTuner.tunable("kS", IntakeConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
  private DashboardNumber m_kv = m_intakePivotTuner.tunable("kV", IntakeConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
  private DashboardNumber m_ka = m_intakePivotTuner.tunable("kA", IntakeConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);


  /** Creates a new Intake. */
  public Intake() {
    // Roller motor config
    m_intakeRollerMotor.Configuration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.RollerSupplyCurrentLimit.in(Amps);
    m_intakeRollerMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_intakeRollerMotor.Configuration.CurrentLimits.StatorCurrentLimit = IntakeConstants.RollerStatorCurrentLimit.in(Amps);
    m_intakeRollerMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    
    m_intakeRollerMotor.Configuration.MotorOutput.Inverted = IntakeConstants.RollerMotorDirection;
    m_intakeRollerMotor.Configuration.MotorOutput.NeutralMode = IntakeConstants.RollerNeutralMode;

    // Kicker motor config
    m_intakeKickerMotor.Configuration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.KickerSupplyCurrentLimit.in(Amps);
    m_intakeKickerMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_intakeKickerMotor.Configuration.CurrentLimits.StatorCurrentLimit = IntakeConstants.KickerStatorCurrentLimit.in(Amps);
    m_intakeKickerMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    
    m_intakeKickerMotor.Configuration.MotorOutput.Inverted = IntakeConstants.KickerMotorDirection;
    m_intakeKickerMotor.Configuration.MotorOutput.NeutralMode = IntakeConstants.KickerNeutralMode;

    // Pivot motor config
    m_intakePivotMotor.Configuration.Feedback.RotorToSensorRatio = IntakeConstants.PivotRotorToSensorRatio;
    m_intakePivotMotor.Configuration.Feedback.SensorToMechanismRatio = IntakeConstants.PivotSensorToMechanismRatio;
    m_intakePivotMotor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // TODO: double check

    m_intakePivotMotor.Configuration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PivotSupplyCurrentLimit.in(Amps);
    m_intakePivotMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_intakePivotMotor.Configuration.CurrentLimits.StatorCurrentLimit = IntakeConstants.PivotStatorCurrentLimit.in(Amps);
    m_intakePivotMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;

    m_intakePivotMotor.Configuration.MotorOutput.Inverted = IntakeConstants.PivotMotorDirection;
    m_intakePivotMotor.Configuration.MotorOutput.NeutralMode = IntakeConstants.PivotNeutralMode;

    var slot0 = new Slot0Configs();
    slot0.kP = m_kp.get();
    slot0.kI = m_ki.get();
    slot0.kD = m_kd.get();
    slot0.kG = m_kg.get();
    slot0.kS = m_ks.get();
    slot0.kV = m_kv.get();
    slot0.kA = m_ka.get();
    slot0.GravityType = GravityTypeValue.Elevator_Static;

    m_intakePivotMotor.Configuration.Slot0 = slot0;

    m_intakePivotMotor.applyConfig();
  }

  /**
   * Sets the speed of the intake between -1 and 1.
   */
  public void setIntakeSpeed(double speed) {
    m_intakeRollerMotor.set(speed);
    m_intakeKickerMotor.set(speed);
  }

  /**
   * Sets the speed of the each intake motor between -1 and 1.
   */
  public void setIntakeSpeed(double roller, double kicker) {
    m_intakeRollerMotor.set(roller);
    m_intakeKickerMotor.set(kicker);
  }

  /**
   * Sets the speed of the pivot motor between -1 and 1.
   */
  public void setPivotSpeed(double speed) {
    m_intakeKickerMotor.set(speed);
  }

  /**
   * Sets the angle of the pivot motor.
   */
  public void setPivotAngle(Angle angle) {
    Angle targetAngle = angle;
    if (targetAngle.gt(IntakeConstants.PivotMaxAngle)) {
      targetAngle = IntakeConstants.PivotMaxAngle;
    } else if (targetAngle.lt(IntakeConstants.PivotMinAngle)) {
      targetAngle = IntakeConstants.PivotMinAngle;
    }

    m_intakeKickerMotor.moveToPosition(targetAngle);
  }

  /**
   * Returns the intake roller speed.
   */
  public double getIntakeRollerSpeed() {
    return m_intakeRollerMotor.get();
  }

  /**
   * Returns the intake kicker speed.
   */
  public double getIntakeKickerSpeed() {
    return m_intakeKickerMotor.get();
  }

  /**
   * Returns the intake pivot speed.
   */
  public double getIntakePivotSpeed() {
    return m_intakePivotMotor.get();
  }

  /**
   * Returns the intake pivot angle.
   */
  public Angle getIntakePivotAngle() {
    return m_intakePivotMotor.getPosition().getValue();
  }

  @Override
  public void deploy() {
    setPivotAngle(IntakeConstants.PivotDeployAngle);
  }

  @Override
  public void retract() {
    setPivotAngle(IntakeConstants.PivotRetractAngle);
  }
}
