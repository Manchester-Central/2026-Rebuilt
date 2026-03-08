// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.interfaces.IClimber;

public class Climber extends SubsystemBase implements IClimber {
  private ChaosTalonFx m_climberMotor = new ChaosTalonFx(ClimberConstants.ClimberCanId, ClimberConstants.ClimberCanBus, ClimberConstants.Config);
  private DigitalInput m_limSwitch = new DigitalInput(ClimberConstants.SensorIndex); // TODO: check input channel
  
  private boolean m_hasTouchedBottom = Robot.isReal() ? false : true;
  private Distance m_targetHeight = ClimberConstants.MinExtension;

  @SuppressWarnings("unused")
  private ChaosTalonFxTuner m_climberTuner = new ChaosTalonFxTuner("Climber/Climber Motor", m_climberMotor).withAllConfigs();

  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor.applyConfig();

    if (Robot.isSimulation()) {
      m_climberMotor.attachMotorSim(ClimberConstants.SimValues);
    }
  }

  public Distance getHeight() {
    return Meters.of(m_climberMotor.getPosition().getValueAsDouble());
  }

  public void setHeight(Distance height) {
    m_targetHeight = height;
    if (m_targetHeight.gt(ClimberConstants.MaxExtension)) {
      m_targetHeight = ClimberConstants.MaxExtension;
    } else if (m_targetHeight.lt(ClimberConstants.MinExtension)) {
      m_targetHeight = ClimberConstants.MinExtension;
    }

    if (m_hasTouchedBottom) {
      // m_climberMotor.moveToPosition(m_targetHeight.in(Meters));
    }
  }

  public void setClimberSpeed(double speed) {
    //if (getHeight().lt(ClimberConstants.MinExtension) && speed < 0) {
     // speed = 0;
   // } else if (getHeight().gt(ClimberConstants.MaxExtension) && speed > 0) {
   //   speed = 0;
    //}

   if (m_hasTouchedBottom) {
      // m_climberMotor.set(speed);
    }
  }

  public boolean getClimberAtBottom() {
    return !m_limSwitch.get(); // Magnetic limit switch is false when the climber is at the bottom
  }

  public double getClimberSpeed() {
    return m_climberMotor.get();
  }

  @Override
  public void periodic() {
    if (!m_hasTouchedBottom && DriverStation.isEnabled()) {
      // m_climberMotor.set(ClimberConstants.NotTouchedBottomSpeed.get());
    }

    if (getClimberAtBottom() && !m_hasTouchedBottom) {
      m_climberMotor.setPosition(0);
      m_hasTouchedBottom = true;
    }

    Logger.recordOutput("Climber/hasTouchedBottom", m_hasTouchedBottom);
    Logger.recordOutput("Climber/climberAtBottom", getClimberAtBottom());
    Logger.recordOutput("Climber/HeightMeters", getHeight().in(Meters));
    Logger.recordOutput("Climber/TargetHeightMeters", m_targetHeight.in(Meters));
  }
}
