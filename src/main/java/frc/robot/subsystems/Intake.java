// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.util.ChaosTalonFx;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private ChaosTalonFx m_intakeMotor = new ChaosTalonFx(IntakeConstants.IntakeCanId, IntakeConstants.IntakeCanBus);

  /** Creates a new Intake. */
  public Intake() {
  }

  /**
   * Sets the speed of the intake between -1 and 1.
   */
  public void setIntakeSpeed(double speed) {
    m_intakeMotor.setSpeed(speed);
  }

  /**
   * Returns the intake speed.
   */
  public double getIntakeSpeed() {
    return m_intakeMotor.get();
  }
}
