// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.chaos131.ctre.ChaosCanCoder;
import com.chaos131.ctre.ChaosCanCoderTuner;
import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.RollerConstants;
import frc.robot.constants.IntakeConstants.PivotConstants;
import frc.robot.subsystems.interfaces.IIntake;

public class Intake extends SubsystemBase implements IIntake {
  private ChaosTalonFx m_rollerMotor = new ChaosTalonFx(RollerConstants.RollerCanId, IntakeConstants.CanBus);
  // private ChaosTalonFx m_intakeKickerMotor = new ChaosTalonFx(IntakeConstants.IntakeKickerCanId, IntakeConstants.IntakeCanBus); TODO: delete if not added to robot
  private ChaosTalonFx m_pivotMotor = new ChaosTalonFx(PivotConstants.PivotCanId, IntakeConstants.CanBus);
  private ChaosTalonFxTuner m_pivotTuner = new ChaosTalonFxTuner("PivotTuner", m_pivotMotor);
  private ChaosCanCoder m_pivotCanCoder = new ChaosCanCoder(PivotConstants.PivotCanCoderId, IntakeConstants.CanBus);
  private ChaosCanCoderTuner m_pivotCanCoderTuner = new ChaosCanCoderTuner("PivotCanCoderTuner", m_pivotCanCoder);

  private DashboardNumber m_canCoderOffsetDegrees = m_pivotCanCoderTuner.tunable("CANCoder Tuner",
      PivotConstants.CanCoderOffset.in(Degrees), (config, newValue) -> 
      config.MagnetSensor.MagnetOffset = Degrees.of(newValue).in(Rotations));

  private DashboardNumber m_kp = m_pivotTuner.tunable("kP", PivotConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
  private DashboardNumber m_ki = m_pivotTuner.tunable("kI", PivotConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
  private DashboardNumber m_kd = m_pivotTuner.tunable("kD", PivotConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
  private DashboardNumber m_kg = m_pivotTuner.tunable("kG", PivotConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
  private DashboardNumber m_ks = m_pivotTuner.tunable("kS", PivotConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
  private DashboardNumber m_kv = m_pivotTuner.tunable("kV", PivotConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
  private DashboardNumber m_ka = m_pivotTuner.tunable("kA", PivotConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);

  /** Creates a new Intake. */
  public Intake() {
    // Roller motor config
    m_rollerMotor.Configuration.CurrentLimits.SupplyCurrentLimit = RollerConstants.SupplyCurrentLimit.in(Amps);
    m_rollerMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_rollerMotor.Configuration.CurrentLimits.StatorCurrentLimit = RollerConstants.StatorCurrentLimit.in(Amps);
    m_rollerMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    
    m_rollerMotor.Configuration.MotorOutput.Inverted = RollerConstants.MotorDirection;
    m_rollerMotor.Configuration.MotorOutput.NeutralMode = RollerConstants.NeutralMode;

    // Kicker motor config
    // m_intakeKickerMotor.Configuration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.KickerSupplyCurrentLimit.in(Amps);
    // m_intakeKickerMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // m_intakeKickerMotor.Configuration.CurrentLimits.StatorCurrentLimit = IntakeConstants.KickerStatorCurrentLimit.in(Amps);
    // m_intakeKickerMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // m_intakeKickerMotor.Configuration.MotorOutput.Inverted = IntakeConstants.KickerMotorDirection;
    // m_intakeKickerMotor.Configuration.MotorOutput.NeutralMode = IntakeConstants.KickerNeutralMode;

    // Pivot motor config
    m_pivotMotor.Configuration.Feedback.RotorToSensorRatio = PivotConstants.RotorToSensorRatio;
    m_pivotMotor.Configuration.Feedback.SensorToMechanismRatio = PivotConstants.SensorToMechanismRatio;
    m_pivotMotor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // TODO: double check

    m_pivotMotor.Configuration.CurrentLimits.SupplyCurrentLimit = PivotConstants.SupplyCurrentLimit.in(Amps);
    m_pivotMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_pivotMotor.Configuration.CurrentLimits.StatorCurrentLimit = PivotConstants.StatorCurrentLimit.in(Amps);
    m_pivotMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;

    m_pivotMotor.Configuration.MotorOutput.Inverted = PivotConstants.MotorDirection;
    m_pivotMotor.Configuration.MotorOutput.NeutralMode = PivotConstants.NeutralMode;

    // Pivot CanCoder config
    m_pivotCanCoder.Configuration.MagnetSensor.SensorDirection = PivotConstants.CanCoderDirection;
    m_pivotCanCoder.Configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = PivotConstants.CanCoderDiscontinuityPoint.in(Rotations);
    m_pivotCanCoder.Configuration.MagnetSensor.MagnetOffset = Degrees.of(m_canCoderOffsetDegrees.get()).in(Rotations);

    m_pivotCanCoder.applyConfig();

    m_pivotMotor.attachCanCoderSim(m_pivotCanCoder);

    var slot0 = new Slot0Configs();
    slot0.kP = m_kp.get();
    slot0.kI = m_ki.get();
    slot0.kD = m_kd.get();
    slot0.kG = m_kg.get();
    slot0.kS = m_ks.get();
    slot0.kV = m_kv.get();
    slot0.kA = m_ka.get();
    slot0.GravityType = GravityTypeValue.Elevator_Static;

    m_pivotMotor.Configuration.Slot0 = slot0;

    m_pivotMotor.applyConfig();

    if (Robot.isSimulation()) {
      var m_moi = SingleJointedArmSim.estimateMOI(IntakeConstants.IntakeLength.in(Meters), IntakeConstants.IntakeMass.in(Kilograms));
      var m_dcMotor = DCMotor.getKrakenX60(1); // TODO: double check
      var m_dcMotorSim = new DCMotorSim(LinearSystemId.createSingleJointedArmSystem(m_dcMotor, m_moi, PivotConstants.RotorToSensorRatio), m_dcMotor);
      m_pivotMotor.attachMotorSim(m_dcMotorSim, PivotConstants.SensorToMechanismRatio, true, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
    }
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
