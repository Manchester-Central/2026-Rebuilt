// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.SplineEncoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.PivotConstants;
import frc.robot.constants.IntakeConstants.RollerConstants;

public class Intake extends SubsystemBase {
  private ChaosTalonFx m_rollerMotor = new ChaosTalonFx(RollerConstants.RollerCanId, IntakeConstants.CanBus, RollerConstants.Config);
  // private ChaosTalonFx m_intakeKickerMotor = new ChaosTalonFx(IntakeConstants.IntakeKickerCanId, IntakeConstants.IntakeCanBus); TODO: delete if not added to robot
  private ChaosTalonFx m_pivotMotor = new ChaosTalonFx(PivotConstants.PivotCanId, IntakeConstants.CanBus, PivotConstants.TalonConfig);
  private SplineEncoder m_pivotEncoder = new SplineEncoder(PivotConstants.PivotCanCoderId.id);

  @SuppressWarnings("unused")
  private ChaosTalonFxTuner m_rollerTuner = new ChaosTalonFxTuner("Intake/Roller Motor", m_rollerMotor).withCurrentLimits();
  @SuppressWarnings("unused")
  private ChaosTalonFxTuner m_pivotTuner = new ChaosTalonFxTuner("Intake/Pivot Motor", m_pivotMotor).withAllConfigs();
  private Angle m_targetAngle = PivotConstants.RetractAngle.get();

  /** Creates a new Intake. */
  public Intake() {

    m_pivotMotor.applyConfig();
    m_rollerMotor.applyConfig();

    m_pivotEncoder.configure(PivotConstants.pivotEncoderConfig, ResetMode.kResetSafeParameters);

    if (Robot.isSimulation()) {
      m_pivotMotor.attachMotorSim(PivotConstants.SimValues);
      m_pivotMotor.setSimAngle(PivotConstants.MinAngle);
    }

    m_pivotMotor.setPosition(getAbsolutePivotAngle());
  }

  /**
   * Sets the speed of the intake between -1 and 1.
   */
  public void setRollerSpeed(double speed) {
    m_rollerMotor.set(speed);
    // m_intakeKickerMotor.set(speed);
  }

  /**
   * Sets the speed of the each intake motor between -1 and 1.
   */
  // public void setIntakeSpeed(double roller, double kicker) {
  //   m_intakeRollerMotor.set(roller);
  //   m_intakeKickerMotor.set(kicker);
  // }

  /**
   * Sets the speed of the pivot motor between -1 and 1.
   */
  public void setPivotSpeed(double speed) {
    double targetSpeed = speed;
    if (getPivotAngle().gt(PivotConstants.MaxAngle)) {
      targetSpeed = Math.min(speed, 0);
    } else if (getPivotAngle().lt(PivotConstants.MinAngle)) {
      targetSpeed = Math.max(speed, 0);
    }

    m_pivotMotor.set(targetSpeed); // TODO: Change to targetSpeed
  }

  /**
   * Sets the angle of the pivot motor.
   */
  public void setPivotAngle(Angle angle) {
    m_targetAngle = angle;
    if (m_targetAngle.gt(PivotConstants.MaxAngle)) {
      m_targetAngle = PivotConstants.MaxAngle;
    } else if (m_targetAngle.lt(PivotConstants.MinAngle)) {
      m_targetAngle = PivotConstants.MinAngle;
    }

    m_pivotMotor.moveToPosition(m_targetAngle);
  }

  /**
   * Returns the intake roller speed.
   */
  public double getRollerSpeed() {
    return m_rollerMotor.get();
  }

  /**
   * Returns the intake kicker speed.
   */
  // public double getIntakeKickerSpeed() {
  //   return m_intakeKickerMotor.get();
  // }

  /**
   * Returns the intake pivot speed.
   */
  public double getIntakePivotSpeed() {
    return m_pivotMotor.get();
  }

  /**
   * Returns the intake pivot angle.
   */
  public Angle getPivotAngle() {
    return m_pivotMotor.getPosition().getValue();
  }

  /**
   * Returns pivot encoder angle
   */
  public Angle getAbsolutePivotAngle() {
    return Rotations.of(m_pivotEncoder.getAngle());
  }

  public void deploy() {
    setPivotAngle(PivotConstants.DeployAngle.get());
  }

  public void retract() {
    setPivotAngle(PivotConstants.RetractAngle.get());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/PivotAngleDegrees", getPivotAngle().in(Degrees));
    Logger.recordOutput("Intake/AbsolutePivotAngleDegrees", getAbsolutePivotAngle().in(Degrees));
    Logger.recordOutput("Intake/RollerSpeed", getRollerSpeed());
    Logger.recordOutput("intake/targetAngle", m_targetAngle.in(Degrees));
  }
}
