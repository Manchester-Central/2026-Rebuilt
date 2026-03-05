// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldDimensions;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.interfaces.IDrive;
import frc.robot.subsystems.interfaces.IFeeder;
import frc.robot.subsystems.interfaces.IFlywheel;
import frc.robot.subsystems.interfaces.IHood;
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

  @Override
  public void setHoodAngle(Angle targetAngle) {
    m_hood.setHoodAngle (targetAngle); 
  }

  @Override
  public Angle getHoodAngle() {
    return m_hood.getHoodAngle();
  }

  public boolean atTargetHoodAngle() {
    return m_hood.atTargetHoodAngle();
  }

  @Override
  public boolean doesFeederHaveFuel() {
    return m_feeder.doesFeederHaveFuel();
  }


  /**
   * Determines launch velocity needed to hit a certain target from any position asumming a constant launch angle.
   * 
   * @param currentPose The pose of the robot when launching.
   * @param targetPose The pose of the target to launch.
   * @param targetHeight The height to target at the specified pose.
   * @return The linear velocity needed to launch at the target.
   */
  private LinearVelocity getVelocityForTargetSetAngle(Pose2d currentPose, Pose2d targetPose, Distance targetHeight) {
    double groundDisplacementInches = FieldPose.getDistanceFromLocations(currentPose, targetPose).in(Inches);
    double thetaRadians = LauncherConstants.LauncherAngle.in(Radians);
    double deltaHeightInches = targetHeight.minus(LauncherConstants.LauncherHeight).in(Inches);
    
    double numerator = GeneralConstants.gravity.in(InchesPerSecondPerSecond) * Math.pow(groundDisplacementInches, 2);
    double denomenator = 2 * Math.pow(Math.cos(thetaRadians), 2) * (deltaHeightInches - groundDisplacementInches * Math.tan(thetaRadians));

    double velocity = Math.sqrt(numerator / denomenator);
    return InchesPerSecond.of(velocity);
  }

  /**
   * Determines launch velocity needed to score in the hub from any position asumming a constant launch angle.
   * 
   * @param currentPose The pose of the robot when launching.
   * @return The linear velocity needed to launch in the hub.
   */
  public LinearVelocity getScoringVelocitySetAngle(Pose2d currentPose) {
    return getVelocityForTargetSetAngle(currentPose, FieldPose2026.HubCenter.getCurrentAlliancePose(), FieldDimensions.HubHeight).times(4);
  }

  /**
   * Determines launch velocity needed to pass to a pass point from any position asumming a constant launch angle.
   * 
   * @param currentPose The pose of the robot when launching.
   * @return The linear velocity needed to launch at the closest pass point.
   */
  public LinearVelocity getPassVelocitySetAngle(Pose2d currentPose) {
    FieldPose closestPose = FieldPose.getClosestPose(currentPose, LauncherConstants.PassPoints);
    return getVelocityForTargetSetAngle(currentPose, closestPose.getCurrentAlliancePose(), Inches.of(0));
  }


  /**
   * Determines the time it will take a fuel to reach a target once it is launched.
   * 
   * @param targetHeight Height of the target that will be launched at.
   * @return The time it will take the fuel to reach its target.
   */
  private Time getFuelFlightTime(Distance targetHeight) {
    Distance deltaMaxHeight = LauncherConstants.MaxLaunchHeight.get().minus(LauncherConstants.LauncherHeight);
    Distance deltatargetHeight = targetHeight.minus(LauncherConstants.LauncherHeight);
    
    Time flightTime = Seconds.of(
      (Math.sqrt(19.62 * deltaMaxHeight.in(Meters)) + Math.sqrt(19.62 * (deltaMaxHeight.minus(deltatargetHeight).in(Meters))))
      / (Math.abs(GeneralConstants.gravity.in(MetersPerSecondPerSecond))));
    
    return flightTime;
  }

  /**
   * Determines the launch velocity needed to launch to a set target from any position at a set max height. Does account for shoot on the move.
   * 
   * @param swerveDrive
   * @param targetPose The pose of the target to launch.
   * @param targetHeight The height to target at the specified pose.
   * @return The linear velocity needed to launch at the target.
   */
  public LinearVelocity getVelocityForTargetSetHeight(IDrive swerveDrive, Pose2d targetPose, Distance targetHeight) {
    Pose2d currentPose = swerveDrive.getPose();
    
    double deltaXMeters = FieldPose.getDeltaXFromLocations(currentPose, targetPose).in(Meters);
    double deltaYMeters = FieldPose.getDeltaYFromLocations(currentPose, targetPose).in(Meters);

    double swerveVelocityXMPS = swerveDrive.getVelocityVector().getX();
    double swerveVelocityYMPS = swerveDrive.getVelocityVector().getY();

    double deltaMaxHeightMeters = LauncherConstants.MaxLaunchHeight.get().minus(LauncherConstants.LauncherHeight).in(Meters);

    double timeSeconds = getFuelFlightTime(targetHeight).in(Seconds);
    
    LinearVelocity targetVelocity = MetersPerSecond.of(
      Math.sqrt(
        Math.pow((deltaXMeters / timeSeconds) - swerveVelocityXMPS, 2)
        + Math.pow((deltaYMeters / timeSeconds) - swerveVelocityYMPS, 2)
        + 19.62 * deltaMaxHeightMeters));

    return targetVelocity;
  }

  /**
   * Determines the vertical launch angle of the fuel needed to launch to a set target from any position at a set max height. Does account for shoot on the move.
   * 
   * @param swerveDrive
   * @param targetPose The pose of the target to launch.
   * @param targetHeight The height to target at the specified pose.
   * @return The pitch needed to launch at the target. (For adjustable hood)
   */
  public Angle getPitchForTarget(IDrive swerveDrive, Pose2d targetPose, Distance targetHeight) {
    Pose2d currentPose = swerveDrive.getPose();
    
    double deltaXMeters = FieldPose.getDeltaXFromLocations(currentPose, targetPose).in(Meters);
    double deltaYMeters = FieldPose.getDeltaYFromLocations(currentPose, targetPose).in(Meters);

    double swerveVelocityXMPS = swerveDrive.getVelocityVector().getX();
    double swerveVelocityYMPS = swerveDrive.getVelocityVector().getY();

    double deltaMaxHeightMeters = LauncherConstants.MaxLaunchHeight.get().minus(LauncherConstants.LauncherHeight).in(Meters);

    double timeSeconds = getFuelFlightTime(targetHeight).in(Seconds);

    Angle targetPitch = Radians.of(
      Math.atan2(Math.sqrt(19.62 * deltaMaxHeightMeters)
      , Math.sqrt(
        Math.pow((deltaXMeters / timeSeconds) - swerveVelocityXMPS, 2)
        + Math.pow((deltaYMeters / timeSeconds) - swerveVelocityYMPS, 2))));
    
    return targetPitch;
  }

  /**
   * Determines the horizontal launch angle of the fuel needed to launch to a set target from any position at a set max height. Does account for shoot on the move.
   * 
   * @param swerveDrive
   * @param targetPose The pose of the target to launch.
   * @param targetHeight The height to target at the specified pose.
   * @return The yaw needed to launch at the target. (For swerve drive)
   */
  public Angle getYawForTarget(IDrive swerveDrive, Pose2d targetPose, Distance targetHeight) {
    Pose2d currentPose = swerveDrive.getPose();
    
    double deltaXMeters = FieldPose.getDeltaXFromLocations(currentPose, targetPose).in(Meters);
    double deltaYMeters = FieldPose.getDeltaYFromLocations(currentPose, targetPose).in(Meters);

    double swerveVelocityXMPS = swerveDrive.getVelocityVector().getX();
    double swerveVelocityYMPS = swerveDrive.getVelocityVector().getY();

    double timeSeconds = getFuelFlightTime(targetHeight).in(Seconds);

    Angle targetYaw = Radians.of(
      Math.atan2(
        deltaYMeters - timeSeconds * swerveVelocityYMPS, 
        deltaXMeters - timeSeconds * swerveVelocityXMPS));
    
    return targetYaw;
  }
 } 
