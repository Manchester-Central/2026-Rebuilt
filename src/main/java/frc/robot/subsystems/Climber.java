// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.util.ChaosTalonFx;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {}

 private ChaosTalonFx m_climberMotor = new ChaosTalonFx(ClimberConstants.ClimberCanId, ClimberConstants.ClimberCanBus);

 public void setClimberSpeed (double speed) {
        m_climberMotor.setSpeed(speed);
    }
    
    public double getClimberSpeed () {
        return m_climberMotor.get();
    }
}
