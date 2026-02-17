// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosCanCoder;
import com.chaos131.ctre.ChaosCanCoderTuner;
import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.SplineEncoder;
import com.revrobotics.encoder.config.DetachedEncoderConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ArenaConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ArenaConstants.MotorIDs;
import frc.robot.constants.GeneralConstants.Mode;
import frc.robot.constants.IntakeConstants.RollerConstants;
import frc.robot.constants.IntakeConstants.PivotConstants;
import frc.robot.subsystems.interfaces.IIntake;

public class Intake extends SubsystemBase implements IIntake {
  protected ChaosTalonFx m_rollerMotor;
  // private ChaosTalonFx m_intakeKickerMotor = new ChaosTalonFx(IntakeConstants.IntakeKickerCanId, IntakeConstants.IntakeCanBus); TODO: delete if not added to robot
  protected ChaosTalonFx m_pivotMotor;
  // private SplineEncoder m_pivotEncoder = new SplineEncoder(PivotConstants.PivotCanCoderId.id);

  @SuppressWarnings("unused")
  protected ChaosTalonFxTuner m_rollerTuner;
  @SuppressWarnings("unused")
  protected ChaosTalonFxTuner m_pivotTuner;

  /** Creates a new Intake. */
  public Intake(int id) {
    if (GeneralConstants.currentMode != Mode.ARENA) {
      m_rollerMotor = new ChaosTalonFx(RollerConstants.RollerCanId, IntakeConstants.CanBus, RollerConstants.Config);
      m_pivotMotor = new ChaosTalonFx(PivotConstants.PivotCanId, IntakeConstants.CanBus, PivotConstants.TalonConfig);
    } else {
      m_rollerMotor = new ChaosTalonFx(ArenaConstants.motorCanIDs[id][MotorIDs.Intake.canIdx], IntakeConstants.CanBus, RollerConstants.Config);
      m_pivotMotor = new ChaosTalonFx(ArenaConstants.motorCanIDs[id][MotorIDs.IntakePivot.canIdx], IntakeConstants.CanBus, PivotConstants.TalonConfig);
    }

    m_rollerTuner = new ChaosTalonFxTuner("Intake/Roller Motor", m_rollerMotor).withCurrentLimits();
    m_pivotTuner = new ChaosTalonFxTuner("Intake/Pivot Motor", m_pivotMotor).withAllConfigs();
    m_pivotMotor.applyConfig();
    m_rollerMotor.applyConfig();

    // m_pivotEncoder.configure(PivotConstants.pivotEncoderConfig, ResetMode.kResetSafeParameters);

    if (Robot.isSimulation()) {
      var m_moi = SingleJointedArmSim.estimateMOI(IntakeConstants.IntakeLength.in(Meters), IntakeConstants.IntakeMass.in(Kilograms));
      var m_dcMotor = DCMotor.getKrakenX60(1); // TODO: double check
      var m_dcMotorSim = new DCMotorSim(LinearSystemId.createSingleJointedArmSystem(m_dcMotor, m_moi, PivotConstants.SensorToMechanismRatio), m_dcMotor);
      m_pivotMotor.attachMotorSim(m_dcMotorSim, PivotConstants.SensorToMechanismRatio, true, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
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

    // m_pivotMotor.set(targetSpeed);
  }

  /**
   * Sets the angle of the pivot motor.
   */
  public void setPivotAngle(Angle angle) {
    Angle targetAngle = angle;
    if (targetAngle.gt(PivotConstants.MaxAngle)) {
      targetAngle = PivotConstants.MaxAngle;
    } else if (targetAngle.lt(PivotConstants.MinAngle)) {
      targetAngle = PivotConstants.MinAngle;
    }

    // m_pivotMotor.moveToPosition(targetAngle);
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

  public boolean atTargetAngle(Angle targetAngle) {
    if (getPivotAngle().isNear(targetAngle, Degrees.of(10))) {
      return true;
    }

    return false;
  }

  /**
   * Returns pivot encoder angle
   */
  public Angle getAbsolutePivotAngle() {
    return Rotations.of(0); // Rotations.of(m_pivotEncoder.getAngle());
  }

  @Override
  public void deploy() {
    setPivotAngle(PivotConstants.DeployAngle);
  }

  @Override
  public void retract() {
    setPivotAngle(PivotConstants.RetractAngle);
  }

  @Override
  public void simulationPeriodic() {
    // m_pivotMotor.simulationPeriodic();
  }

  @Override
  public int getNumGamePieces() {
    return 1;
  }

  @Override
  public boolean claimGamePiece() {
    return true;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Angle", getPivotAngle());
    Logger.recordOutput("Intake/AbsoluteAngle", getAbsolutePivotAngle());
  }
}
