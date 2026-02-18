// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.chaos131.poses.FieldPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.subsystems.interfaces.ILauncher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetPassVelocityAndLaunch extends Command {
  ILauncher m_launcher;
  Supplier<Pose2d> m_currentPoseSupplier;
  /** Creates a new AimHubAndLaunch. */
  public TargetPassVelocityAndLaunch(ILauncher launcher, Supplier<Pose2d> currentPoseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_launcher = launcher;
    m_currentPoseSupplier = currentPoseSupplier;

    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private boolean isFacingTarget() {
    Pose2d currentPose = m_currentPoseSupplier.get();
    Angle currentAngle = currentPose.getRotation().getMeasure();
    FieldPose targetPose = FieldPose.getClosestPose(m_currentPoseSupplier.get(), LauncherConstants.PassPoints);
    Angle targetAngle = targetPose.getTargetAngleForRobot(currentPose).getMeasure();

    Logger.recordOutput("Launcher/TargetAngle", targetAngle.in(Degrees));
    Logger.recordOutput("Launcher/CurrentAngle", currentAngle.in(Degrees));
    Logger.recordOutput("Launcher/AngleDif", targetAngle.minus(currentAngle).in(Degrees));
    return targetAngle.isNear(currentAngle, LauncherConstants.AimYawTolerance.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.setFlywheelVelocity(m_launcher.getPassVelocity(m_currentPoseSupplier.get()));
    if (m_launcher.atTargetFlywheelVelocity() && isFacingTarget()) {
      m_launcher.setFeederSpeed(FeederConstants.FeederSpeed.get());
    } else {
      m_launcher.setFeederSpeed(0);
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
