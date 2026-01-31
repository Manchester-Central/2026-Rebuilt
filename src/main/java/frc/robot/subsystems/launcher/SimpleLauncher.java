// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ISimpleLauncher;

public class SimpleLauncher extends SubsystemBase implements ISimpleLauncher {
  Flywheel m_flywheel;

  /** Creates a new SimpleLauncher. */
  public SimpleLauncher(Flywheel flywheel) {
    m_flywheel = flywheel;
  }

  public double getFlywheelSpeed() {
    return m_flywheel.getFlywheelSpeed();
  }

  public void setFlywheelSpeed(double speed) {
    m_flywheel.setFlywheelSpeed(speed);
  }

  public LinearVelocity getFlywheelVelocity() {
    return MetersPerSecond.of(m_flywheel.getFlywheelSpeed());
  }

  public LinearVelocity getScoringVelocity(Pose2d currentPose) {
    return MetersPerSecond.of(0);
  }

  public LinearVelocity getMidPassVelocity(Pose2d currentPose) {
    return MetersPerSecond.of(0);
  }

  public LinearVelocity getFarPassVelocity(Pose2d currentPose) {
    return MetersPerSecond.of(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
