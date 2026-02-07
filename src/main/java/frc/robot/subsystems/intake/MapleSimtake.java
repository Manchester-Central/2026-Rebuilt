// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.chaos131.ctre.ChaosCanCoder;
import com.chaos131.ctre.ChaosCanCoderTuner;
import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.RollerConstants;
import frc.robot.constants.IntakeConstants.PivotConstants;

public class MapleSimtake extends Intake {
  private ChaosTalonFx m_rollerMotor = new ChaosTalonFx(RollerConstants.RollerCanId, IntakeConstants.CanBus, RollerConstants.Config);
  // private ChaosTalonFx m_intakeKickerMotor = new ChaosTalonFx(IntakeConstants.IntakeKickerCanId, IntakeConstants.IntakeCanBus); TODO: delete if not added to robot
  private ChaosTalonFx m_pivotMotor = new ChaosTalonFx(PivotConstants.PivotCanId, IntakeConstants.CanBus, PivotConstants.TalonConfig);
  private ChaosTalonFxTuner m_pivotTuner = new ChaosTalonFxTuner("PivotTuner", m_pivotMotor);
  private ChaosCanCoder m_pivotCanCoder = new ChaosCanCoder(PivotConstants.PivotCanCoderId, IntakeConstants.CanBus, PivotConstants.CanCoderConfig);
  private ChaosCanCoderTuner m_pivotCanCoderTuner = new ChaosCanCoderTuner("PivotCanCoderTuner", m_pivotCanCoder);

  /** Creates a new Intake. */
  public MapleSimtake(int id) {
    super(id);
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

    m_pivotMotor.set(targetSpeed);
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

    m_pivotMotor.moveToPosition(targetAngle);
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
    return m_pivotCanCoder.getAbsolutePosition().getValue();
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
    m_pivotMotor.simulationPeriodic();
  }
}
