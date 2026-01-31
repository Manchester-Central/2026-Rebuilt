// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import com.chaos131.util.ChaosTalonFx;
import com.chaos131.util.ChaosTalonFxTuner;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.interfaces.IClimber;

public class Climber extends SubsystemBase implements IClimber {
  private ChaosTalonFx m_climberMotor = new ChaosTalonFx(ClimberConstants.ClimberCanId, ClimberConstants.ClimberCanBus);
  private ChaosTalonFxTuner m_climberTuner = new ChaosTalonFxTuner("ClimberTuner", m_climberMotor);

  private DashboardNumber m_kp = m_climberTuner.tunable("kP", ClimberConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
  private DashboardNumber m_ki = m_climberTuner.tunable("kI", ClimberConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
  private DashboardNumber m_kd = m_climberTuner.tunable("kD", ClimberConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
  private DashboardNumber m_kg = m_climberTuner.tunable("kG", ClimberConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
  private DashboardNumber m_ks = m_climberTuner.tunable("kS", ClimberConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
  private DashboardNumber m_kv = m_climberTuner.tunable("kV", ClimberConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
  private DashboardNumber m_ka = m_climberTuner.tunable("kA", ClimberConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);

  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor.Configuration.Feedback.RotorToSensorRatio = ClimberConstants.RotorToSensorRatio;
    m_climberMotor.Configuration.Feedback.SensorToMechanismRatio = ClimberConstants.SensorToMechanismRatio;
    m_climberMotor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // TODO: double check

    m_climberMotor.Configuration.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SupplyCurrentLimit.in(Amps);
    m_climberMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_climberMotor.Configuration.CurrentLimits.StatorCurrentLimit = ClimberConstants.StatorCurrentLimit.in(Amps);
    m_climberMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    
    m_climberMotor.Configuration.MotorOutput.Inverted = ClimberConstants.MotorDirection;
    m_climberMotor.Configuration.MotorOutput.NeutralMode = ClimberConstants.NeutralMode;

    var slot0 = new Slot0Configs();
    slot0.kP = m_kp.get();
    slot0.kI = m_ki.get();
    slot0.kD = m_kd.get();
    slot0.kG = m_kg.get();
    slot0.kS = m_ks.get();
    slot0.kV = m_kv.get();
    slot0.kA = m_ka.get();
    slot0.GravityType = GravityTypeValue.Elevator_Static;

    m_climberMotor.Configuration.Slot0 = slot0;

    m_climberMotor.applyConfig();

    if (Robot.isSimulation()) {
      var m_dcMotor = DCMotor.getKrakenX60(1); // TODO: double check
      var m_dcMotorSim = new DCMotorSim(
        LinearSystemId.createElevatorSystem(m_dcMotor, 2, 0.05, ClimberConstants.SensorToMechanismRatio), // TODO: double check
        m_dcMotor
      );
      m_climberMotor.attachMotorSim(m_dcMotorSim, ClimberConstants.SensorToMechanismRatio, true, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
    }
  }

  public Distance getHeight() {
    return Meters.of(m_climberMotor.getPosition().getValueAsDouble());
  }

  public void setHeight(Distance height) {
    Distance targetHeight = height;
    if (targetHeight.gt(ClimberConstants.MaxExtension)) {
      targetHeight = ClimberConstants.MaxExtension;
    } else if (targetHeight.lt(ClimberConstants.MinExtension)) {
      targetHeight = ClimberConstants.MinExtension;
    }

    m_climberMotor.moveToPosition(height.in(Meters));
  }

  public void setClimberSpeed(double speed) {
    if (getHeight().lt(ClimberConstants.MinExtension) && speed < 0) {
      speed = 0;
    } else if (getHeight().gt(ClimberConstants.MaxExtension) && speed > 0) {
      speed = 0;
    }
    m_climberMotor.set(speed);
  }

  public double getClimberSpeed() {
    return m_climberMotor.get();
  }

  @Override
  public void simulationPeriodic() {
    m_climberMotor.simulationPeriodic();
  }
}
