// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.interfaces.IClimber;

public class Climber extends SubsystemBase implements IClimber {
  private ChaosTalonFx m_climberMotor = new ChaosTalonFx(ClimberConstants.ClimberCanId, ClimberConstants.ClimberCanBus, ClimberConstants.Config);
  private DigitalInput m_limSwitch = new DigitalInput(0); // TODO: check input channel
  
  private boolean m_hasTouchedBottom = false;

  @SuppressWarnings("unused")
  private ChaosTalonFxTuner m_climberTuner = new ChaosTalonFxTuner("Climber/Climber Motor", m_climberMotor).withAllConfigs();

  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor.applyConfig();

    if (Robot.isSimulation()) {
      var m_dcMotor = DCMotor.getKrakenX60(1); // TODO: double check
      var m_dcMotorSim = new DCMotorSim(
        LinearSystemId.createElevatorSystem(m_dcMotor, ClimberConstants.ClimberMass.in(Kilogram), ClimberConstants.DrivingDrumRadius.in(Meters), ClimberConstants.SensorToMechanismRatio),
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

    if (m_hasTouchedBottom) {
      m_climberMotor.moveToPosition(height.in(Meters));
    }
  }

  public void setClimberSpeed(double speed) {
    if (getHeight().lt(ClimberConstants.MinExtension) && speed < 0) {
      speed = 0;
    } else if (getHeight().gt(ClimberConstants.MaxExtension) && speed > 0) {
      speed = 0;
    }

    if (m_hasTouchedBottom) {
      m_climberMotor.set(speed);
    }
  }

  public boolean getClimberAtBottom() {
    return m_limSwitch.get();
  }

  public double getClimberSpeed() {
    return m_climberMotor.get();
  }

  @Override
  public void periodic() {
    if (!m_hasTouchedBottom && DriverStation.isEnabled()) {
      m_climberMotor.set(ClimberConstants.NotTouchedBottomSpeed.get());
    }

    if (getClimberAtBottom()) {
      m_climberMotor.setPosition(0);
      m_hasTouchedBottom = true;
    }

    Logger.recordOutput("Climber/hasTouchedBottom", m_hasTouchedBottom);
    Logger.recordOutput("Climber/climberAtBottom", getClimberAtBottom());
  }
}
