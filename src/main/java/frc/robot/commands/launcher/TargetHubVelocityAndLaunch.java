// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.chaos131.poses.FieldPose2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.subsystems.interfaces.ILauncher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetHubVelocityAndLaunch extends Command {
  ILauncher m_launcher;
  Supplier<Pose2d> m_currentPoseSupplier;
  Timer m_launchTimer = new Timer();
  /** Creates a new AimHubAndLaunch. */
  public TargetHubVelocityAndLaunch(ILauncher launcher, Supplier<Pose2d> currentPoseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_launcher = launcher;
    m_currentPoseSupplier = currentPoseSupplier;

    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_launchTimer.stop();
    m_launchTimer.reset();
  }

  private boolean isFacingTarget() {
    Pose2d currentPose = m_currentPoseSupplier.get();
    Angle currentAngle = currentPose.getRotation().getMeasure();
    Angle targetAngle = FieldPose2026.HubCenter.getTargetAngleForRobot(currentPose).getMeasure();

    Logger.recordOutput("Launcher/TargetAngle", targetAngle.in(Degrees));
    Logger.recordOutput("Launcher/CurrentAngle", currentAngle.in(Degrees));
    Logger.recordOutput("Launcher/AngleDif", targetAngle.minus(currentAngle).in(Degrees));
    return targetAngle.isNear(currentAngle, LauncherConstants.AimYawTolerance.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.setFlywheelVelocity(m_launcher.getScoringVelocity(m_currentPoseSupplier.get()));
    if (m_launcher.atTargetFlywheelVelocity() && isFacingTarget()) {
      m_launcher.setFeederSpeed(FeederConstants.FeederSpeed.get());
      m_launchTimer.start();
    } else {
      m_launcher.setFeederSpeed(0);
    }
    Logger.recordOutput("Launcher/TimerSeconds", m_launchTimer.get()); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  if (DriverStation.isAutonomous()){
    return m_launchTimer.get() > LauncherConstants.AutoLaunchTime.get().in(Seconds);
  }
    return false; 
  }
}
