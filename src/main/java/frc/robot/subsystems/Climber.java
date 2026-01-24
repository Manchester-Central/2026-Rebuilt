// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import com.chaos131.util.ChaosTalonFx;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private ChaosTalonFx m_climberMotor = new ChaosTalonFx(ClimberConstants.ClimberCanId, ClimberConstants.ClimberCanBus);

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

    m_climberMotor.applyConfig();
  }

  public Distance getHeight() {
    return Meters.of(m_climberMotor.getPosition().getValueAsDouble());
  }

  public void setClimberSpeed(double speed) {
    if (getHeight().lt(ClimberConstants.MinExtension) && speed < 0) {
      speed = 0;
    } else if (getHeight().gt(ClimberConstants.MaxExtension) && speed > 0) {
      speed = 0;
    }
    m_climberMotor.setSpeed(speed);
  }

  public double getClimberSpeed() {
    return m_climberMotor.get();
  }
}
