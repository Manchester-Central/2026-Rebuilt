// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

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
import frc.robot.subsystems.interfaces.IHood;
import frc.robot.subsystems.interfaces.IFeeder;
import frc.robot.subsystems.interfaces.ILauncher;

public class Launcher extends SubsystemBase implements ILauncher {
  IFlywheel m_flywheel;
  IFeeder m_feeder;
  IHood m_hood; 


  /** Creates a new Launcher. */
  public Launcher(IFlywheel flywheel, IFeeder feeder, IHood hood) {
    m_flywheel = flywheel;
    m_feeder = feeder;
    m_hood = hood; 
  }

  public double getFlywheelSpeed() {
    return m_flywheel.getFlywheelSpeed();
  }

  public double getFeederSpeed() {
    return m_feeder.getFeederSpeed();
  }

  public double getHoodSpeed() {
    return m_hood.getHoodSpeed();
  }

  public void setHoodSpeed(double speed) {
    m_hood.setHoodSpeed(speed);
  }

  public void setFlywheelSpeed(double speed) {
    m_flywheel.setFlywheelSpeed(speed);
  }

  public void setFeederSpeed(double speed) {
    m_feeder.setFeederSpeed(speed);
  }

  public void setFlywheelVelocity(LinearVelocity velocity) {
    m_flywheel.setFlywheelVelocity(velocity);
  }

  public LinearVelocity getFlywheelVelocity() {
    return m_flywheel.getLinearVelocity(); // TODO: Implement actual velocity getter
  }

  public boolean atTargetFlywheelVelocity() {
    return m_flywheel.atTarget();
  }

  private LinearVelocity getVelocityForTarget(Pose2d currentPose, Pose2d targetPose, Distance targetHeight) {
    double groundDisplacementInches = FieldPose.getDistanceFromLocations(currentPose, targetPose).in(Inches);
    double thetaRadians = LauncherConstants.LauncherAngle.in(Radians);
    double deltaHeightInches = targetHeight.minus(LauncherConstants.LauncherHeight).in(Inches);
    
    double numerator = GeneralConstants.gravity.in(InchesPerSecondPerSecond) * Math.pow(groundDisplacementInches, 2);
    double denomenator = 2 * Math.pow(Math.cos(thetaRadians), 2) * (deltaHeightInches - groundDisplacementInches * Math.tan(thetaRadians));

    double velocity = Math.sqrt(numerator / denomenator);
    return InchesPerSecond.of(velocity);
  }

  public LinearVelocity getScoringVelocity(Pose2d currentPose) {
    return getVelocityForTarget(currentPose, FieldPose2026.HubCenter.getCurrentAlliancePose(), FieldDimensions.HubHeight).times(4);
  }

  public LinearVelocity getPassVelocity(Pose2d currentPose) {
    FieldPose closestPose = FieldPose.getClosestPose(currentPose, LauncherConstants.PassPoints);
    return getVelocityForTarget(currentPose, closestPose.getCurrentAlliancePose(), Inches.of(0));
  }

 }
