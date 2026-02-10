// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldDimensions;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.interfaces.IFlywheel;
import frc.robot.subsystems.interfaces.IIndexer;
import frc.robot.subsystems.interfaces.ISimpleLauncher;

public class SimpleLauncher extends SubsystemBase implements ISimpleLauncher {
  IFlywheel m_flywheel;
  IIndexer m_indexer;

  /** Creates a new SimpleLauncher. */
  public SimpleLauncher(IFlywheel flywheel, IIndexer indexer) {
    m_flywheel = flywheel;
    m_indexer = indexer;
  }

  public double getFlywheelSpeed() {
    return m_flywheel.getFlywheelSpeed();
  }

  public double getIndexerSpeed() {
    return m_indexer.getIndexerSpeed();
  }

  public void setFlywheelSpeed(double speed) {
    m_flywheel.setFlywheelSpeed(speed);
  }

  public void setIndexerSpeed(double speed) {
    m_indexer.setIndexerSpeed(speed);
  }

  public void setFlywheelVelocity(LinearVelocity velocity) {
    m_flywheel.setFlywheelVelocity(velocity);
  }

  public LinearVelocity getFlywheelVelocity() {
    return m_flywheel.getLeftLinearVelocity(); // TODO: Implement actual velocity getter
  }

  public boolean atTargetFlywheelVelocity() {
    return m_flywheel.atTargetRight() && m_flywheel.atTargetLeft();
  }

  private LinearVelocity getVelocityForTarget(Pose2d currentPose, Pose2d targetPose, Distance targetHeight) {
    double groundDisplacementInches = FieldPose.getDistanceFromLocations(currentPose, targetPose).in(Inches);
    double thetaRadians = LauncherConstants.SimpleLauncherAngle.in(Radians);
    double deltaHeightInches = targetHeight.minus(LauncherConstants.SimpleLauncherHeight).in(Inches);
    
    double numerator = GeneralConstants.gravity.in(InchesPerSecondPerSecond) * Math.pow(groundDisplacementInches, 2);
    double denomenator = 2 * Math.pow(Math.cos(thetaRadians), 2) * (deltaHeightInches - groundDisplacementInches * Math.tan(thetaRadians));

    double velocity = Math.sqrt(numerator / denomenator);
    return InchesPerSecond.of(velocity);
  }

  public LinearVelocity getScoringVelocity(Pose2d currentPose) {
    return getVelocityForTarget(currentPose, FieldPose2026.HubCenter.getCurrentAlliancePose(), FieldDimensions.HubHeight);
  }

  public LinearVelocity getPassVelocity(Pose2d currentPose) {
    FieldPose closestPose = FieldPose.getClosestPose(currentPose, LauncherConstants.LeftPassPoint, LauncherConstants.RightPassPoint);
    return getVelocityForTarget(currentPose, closestPose.getCurrentAlliancePose(), Inches.of(0));
  }

  @Override
  public void periodic() {
    m_flywheel.periodic();
  }
}
