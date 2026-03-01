// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.HoodConstants;
import frc.robot.subsystems.interfaces.IHood;

/** Add your docs here. */
public class Hood extends SubsystemBase implements IHood {
 private ChaosTalonFx m_hoodMotor = new ChaosTalonFx (HoodConstants.HoodCanId, LauncherConstants.LauncherCanBus, HoodConstants.HoodConfig); 

  @SuppressWarnings("unused")
  private ChaosTalonFxTuner m_hoodMotorTuner = new ChaosTalonFxTuner ("Launcher/Hood/Hood Motor", m_hoodMotor).withCurrentLimits();

  public Hood(int id) {
    m_hoodMotor.applyConfig(); 
  }

  @Override
  public void setHoodSpeed (double speed) {
        m_hoodMotor.set(speed);
 }

  @Override
  public double getHoodSpeed () {
      return m_hoodMotor.get(); //TODO: finish IHood stuff
 }

  @Override
  public void periodic() {
      Logger.recordOutput("Hood/Speed", getHoodSpeed());
 }
}
