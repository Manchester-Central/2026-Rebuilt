// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.PivotConstants;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.constants.LauncherConstants.FlywheelConstants;
import frc.robot.subsystems.interfaces.IDrive;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.ILauncher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimHubAndLaunchTunable extends Command {
  ILauncher m_launcher;
  IDrive m_swerveDrive;
  IIntake m_intake;

  /** Creates a new AImHubAndLaunchTunable. */
  public AimHubAndLaunchTunable(ILauncher launcher, IDrive swerveDrive, IIntake intake) {
    m_launcher = launcher;
    m_swerveDrive = swerveDrive;
    m_intake = intake;

    addRequirements(m_launcher, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // private boolean isFacingTarget() {
  //   Pose2d currentPose = m_swerveDrive.getPose();
  //   Angle currentAngle = currentPose.getRotation().getMeasure();
  //   Angle targetAngle = m_launcher.getYawForTarget(m_swerveDrive, FieldPose2026.HubCenter.getCurrentAlliancePose(), FieldDimensions.HubHeight);

  //   Logger.recordOutput("Launcher/TargetAngle", targetAngle.in(Degrees));
  //   Logger.recordOutput("Launcher/CurrentAngle", currentAngle.in(Degrees));
  //   Logger.recordOutput("Launcher/AngleDif", targetAngle.minus(currentAngle).in(Degrees));
  //   return targetAngle.isNear(currentAngle, LauncherConstants.AimYawTolerance.get());
  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_launcher.setFlywheelVelocity(FlywheelConstants.TunableLaunchVelocity.get());
    m_intake.setRollerSpeed(IntakeConstants.IntakeRollerSpeed.get());
    m_intake.setPivotAngle(PivotConstants.DeployAngle.get()); // TODO: Testing only

    if (m_launcher.atVelocityDebounced()) {
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
