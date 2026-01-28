// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.util.ChaosTalonFx;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements IIntake {
  private ChaosTalonFx m_intakeRollerMotor = new ChaosTalonFx(IntakeConstants.IntakeRollerCanId, IntakeConstants.IntakeCanBus);
  private ChaosTalonFx m_intakePivotMotor = new ChaosTalonFx(IntakeConstants.IntakePivotCanId, IntakeConstants.IntakeCanBus);
  private ChaosTalonFx m_intakeKickerMotor = new ChaosTalonFx(IntakeConstants.IntakeKickerCanId, IntakeConstants.IntakeCanBus);
  /** Creates a new Intake. */
  public Intake() {
  }

  /**
   * Sets the speed of the intake between -1 and 1.
   */
  public void setIntakeSpeed(double speed) {
    m_intakeRollerMotor.set(speed);
    m_intakeKickerMotor.set(speed);
  }

  /**
   * Returns the intake roller speed.
   */
  public double getIntakeRollerSpeed() {
    return m_intakeRollerMotor.get();
  }

  /**
   * Returns the intake kicker speed.
   */
  public double getIntakeKickerSpeed() {
    return m_intakeKickerMotor.get();
  }
}
