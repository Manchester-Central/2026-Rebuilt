// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;

import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldDimensions;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.LauncherConstants;
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

  private LinearVelocity getVelocityForTarget(Pose2d currentPose, Pose2d targetPose, Distance targetHeight) {
    double groundDisplacementInches = FieldPose.getDistanceFromLocations(currentPose, targetPose).in(Inches);
    double thetaRadians = LauncherConstants.SimpleLauncherAngle.in(Radians);
    double deltaHeightInches = targetHeight.minus(LauncherConstants.SimpleLauncherHeight).in(Inches);

    double velocity = Math.sqrt(
      (GeneralConstants.gravity.in(InchesPerSecondPerSecond) * Math.pow(groundDisplacementInches, 2)) /
      (2 * Math.pow(Math.cos(thetaRadians), 2) * (deltaHeightInches - groundDisplacementInches * Math.tan(thetaRadians))));
    return InchesPerSecond.of(velocity);
  }

  public LinearVelocity getScoringVelocity(Pose2d currentPose) {
    return getVelocityForTarget(currentPose, FieldPose2026.HubCenter.getCurrentAlliancePose(), FieldDimensions.HubHeight);
  }

  public LinearVelocity getPassVelocity(Pose2d currentPose) {
    ArrayList<Pose2d> passPoses = new ArrayList<>();
    passPoses.add(LauncherConstants.LeftPassPoint.getCurrentAlliancePose());
    passPoses.add(LauncherConstants.RightPassPoint.getCurrentAlliancePose());

    return getVelocityForTarget(currentPose, currentPose.nearest(passPoses), Inches.of(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
