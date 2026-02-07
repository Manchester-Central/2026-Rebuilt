// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.chaos131.poses.FieldPose2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.IndexerConstants;
import frc.robot.subsystems.interfaces.ISimpleLauncher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetHubVelocityAndLaunch extends Command {
  ISimpleLauncher m_launcher;
  Supplier<Pose2d> m_currentPoseSupplier;
  /** Creates a new AimHubAndLaunch. */
  public TargetHubVelocityAndLaunch(ISimpleLauncher launcher, Supplier<Pose2d> currentPoseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_launcher = launcher;
    m_currentPoseSupplier = currentPoseSupplier;

    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private boolean isFacingTarget() {
    Translation2d currentAngleTrans2d = m_currentPoseSupplier.get().getTranslation();
    Translation2d targetPoint = FieldPose2026.HubCenter.getCurrentAlliancePose().getTranslation();
    Angle targetAngle = targetPoint.minus(currentAngleTrans2d).getAngle().getMeasure();
    return targetAngle.isNear(currentAngleTrans2d.getAngle().getMeasure(), LauncherConstants.AimYawTolerance.get()); // TODO: Fix in sim
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.setFlywheelVelocity(m_launcher.getScoringVelocity(m_currentPoseSupplier.get()));
    if (m_launcher.atTargetFlywheelVelocity() && isFacingTarget()) {
      m_launcher.setIndexerSpeed(IndexerConstants.IndexerSpeed.get());
    } else {
      m_launcher.setIndexerSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
