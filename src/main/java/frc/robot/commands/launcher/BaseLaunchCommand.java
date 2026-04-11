// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.chaos131.poses.FieldPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.PivotConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interfaces.AbstractDrive;
import frc.robot.subsystems.launcher.Launcher;

/* An abstract class for sharing common launch functions */
public abstract class BaseLaunchCommand extends Command {
  protected Launcher m_launcher;
  protected AbstractDrive m_swerveDrive;
  protected Intake m_intake;
  protected Timer m_launchTimer = new Timer();
  protected boolean m_hasLaunched = false;

  /** Creates a new BaseLaunchCommand. */
  protected BaseLaunchCommand(Launcher launcher, AbstractDrive swerveDrive, Intake intake) {
    m_launcher = launcher;
    m_swerveDrive = swerveDrive;
    m_intake = intake;

    addRequirements(m_launcher, m_intake);
  }

  @Override
  public void initialize() {
    m_hasLaunched = false;
    m_launchTimer.stop();
    m_launchTimer.reset();
  }

  /** This function can be override to allow setting variables before the abstract functions are called in each execute() loop */
  protected void preExecute() {}

  protected boolean isFacingTarget() {
    var targetPose = getTargetPose();
    var targetHeight = getTargetHeight();
    if (targetPose.isEmpty() || targetHeight.isEmpty()) {
      return true;
    }

    Pose2d currentPose = m_swerveDrive.getPose();
    Angle currentAngle = currentPose.getRotation().getMeasure();
    Angle targetAngle = m_launcher.getYawForTarget(m_swerveDrive, targetPose.get().getCurrentAlliancePose(), targetHeight.get());

    Logger.recordOutput("Launcher/TargetAngle", targetAngle.in(Degrees));
    Logger.recordOutput("Launcher/CurrentAngle", currentAngle.in(Degrees));
    Logger.recordOutput("Launcher/AngleDif", targetAngle.minus(currentAngle).in(Degrees));
    return targetAngle.isNear(currentAngle, LauncherConstants.AimYawTolerance.get());
  }

  protected boolean isFacingLaunchTarget() {
    var targetPose = getTargetPose();
    var targetHeight = getTargetHeight();
    if (targetPose.isEmpty() || targetHeight.isEmpty()) {
      return true;
    }

    Pose2d currentPose = m_swerveDrive.getPose();
    Angle currentAngle = currentPose.getRotation().getMeasure();
    Angle targetAngle = m_launcher.getYawForTarget(m_swerveDrive, targetPose.get().getCurrentAlliancePose(), targetHeight.get());

    Logger.recordOutput("Launcher/TargetAngle", targetAngle.in(Degrees));
    Logger.recordOutput("Launcher/CurrentAngle", currentAngle.in(Degrees));
    Logger.recordOutput("Launcher/AngleDif", targetAngle.minus(currentAngle).in(Degrees));
    return targetAngle.isNear(currentAngle, 8.0);
  }

  protected boolean isLauncherReady() {
    return m_launcher.atTargetFlywheelVelocity();
  }

  public Angle getIntakePivotAngle(){
   return PivotConstants.DeployAngle.get(); 
  }


  @Override
  public void execute() {
    preExecute();
    prepLauncher();
    m_intake.setRollerSpeed(IntakeConstants.IntakeRollerSpeed.get());
    m_intake.setPivotAngle(getIntakePivotAngle()); // TODO: Testing only

    if (isFacingTarget() && isLauncherReady()) {
      m_hasLaunched = true;
      m_launchTimer.start();
    }

    if (isFacingLaunchTarget() && m_hasLaunched) {
      enableFeederForLauncher();
    } else {
      m_launcher.setFeederSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.setFlywheelSpeed(0);
    m_launcher.setFeederSpeed(0);
    m_intake.setRollerSpeed(0);
  }

  /**
   * Don't end if the button is still pressed - unless it's auto and we need to time out
   */
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomous()){
      return m_launchTimer.get() > LauncherConstants.AutoLaunchTime.get().in(Seconds);
    }
   
    return false;
  }

  protected abstract Optional<FieldPose> getTargetPose();

  protected abstract Optional<Distance> getTargetHeight();

  protected abstract void prepLauncher();

  protected abstract void enableFeederForLauncher();
}
