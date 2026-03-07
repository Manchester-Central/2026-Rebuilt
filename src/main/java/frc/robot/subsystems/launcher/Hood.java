// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.HoodConstants;
import frc.robot.subsystems.interfaces.IHood;

/** Add your docs here. */
public class Hood extends SubsystemBase implements IHood {
  private ChaosTalonFx m_hoodMotor = new ChaosTalonFx(HoodConstants.HoodCanId, LauncherConstants.LauncherCanBus, HoodConstants.HoodConfig);
  private DigitalInput m_limSwitch = new DigitalInput(HoodConstants.SensorIndex); // TODO: check input channel

  private boolean m_hasReachedMax = false;
  private Angle m_targetAngle = HoodConstants.HoodMaxAngle;

  @SuppressWarnings("unused")
  private ChaosTalonFxTuner m_hoodMotorTuner = new ChaosTalonFxTuner("Launcher/Hood/Hood Motor", m_hoodMotor)
      .withAllConfigs();

  public Hood() {
    m_hoodMotor.applyConfig();
    if (Robot.isSimulation()) {
      m_hoodMotor.attachMotorSim(HoodConstants.SimValues);
      m_hoodMotor.setSimAngle(HoodConstants.HoodMinAngle);
    }
  }

  @Override
  public void setHoodSpeed(double speed) {
    // if (getHoodAngle().gt(HoodConstants.HoodMaxAngle)) {
    //   speed = Math.min(speed, 0);
    // } else if (getHoodAngle().lt(HoodConstants.HoodMinAngle)) {
    //   speed = Math.max(speed, 0);
    // }
    
    m_hoodMotor.set(speed);
  }

  @Override
  public double getHoodSpeed() {
    return m_hoodMotor.get(); // TODO: finish IHood stuff
  }

  @Override
  public void setHoodAngle(Angle targetAngle) {
    m_targetAngle = targetAngle;
    if (m_targetAngle.gt(HoodConstants.HoodMaxAngle)) {
      m_targetAngle = HoodConstants.HoodMaxAngle;
    } else if (m_targetAngle.lt(HoodConstants.HoodMinAngle)) {
      m_targetAngle = HoodConstants.HoodMinAngle;
    }

    m_hoodMotor.moveToPosition(m_targetAngle);

    // if (m_hasReachedMax) {
    //   m_hoodMotor.moveToPosition(m_targetAngle);
    // }
  }

  @Override
  public Angle getHoodAngle() {
    return m_hoodMotor.getPosition().getValue();
  }

  public boolean getHoodAtMax() {
    return !m_limSwitch.get(); // Magnetic limit switch is false when the climber is at the bottom
  }

  @Override
  public boolean atTargetHoodAngle() {
    return getHoodAngle().isNear(m_targetAngle, HoodConstants.TargetAngleTolerance.get());
  }

  @Override
  public void periodic() {
    if (!m_hasReachedMax && DriverStation.isEnabled()) {
      // m_hoodMotor.set(HoodConstants.NotReachedMaxSpeed.get());
    }

    if (getHoodAtMax() && !m_hasReachedMax) {
      m_hoodMotor.setPosition(HoodConstants.HoodMaxAngle);
      m_hasReachedMax = true;
    }

    Logger.recordOutput("Hood/Speed", getHoodSpeed());
    Logger.recordOutput("Hood/Angle_Deg", getHoodAngle().in(Degrees));
    Logger.recordOutput("Hood/TargetAngle_Deg", m_targetAngle.in(Degrees));
  }
}
