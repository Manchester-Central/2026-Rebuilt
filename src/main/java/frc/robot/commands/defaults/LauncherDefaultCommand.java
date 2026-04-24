// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LauncherConstants.HoodConstants;
import frc.robot.subsystems.launcher.Launcher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LauncherDefaultCommand extends Command {
  /** Creates a new SimpleLauncherDefaultCommand. */
  private Launcher m_launcher;

  public LauncherDefaultCommand(Launcher launcher) {
    // Use addRequirements() here to declare subsystem dependencies.=
    m_launcher = launcher;
    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: determine logic while in auto mode
    m_launcher.setFlywheelSpeed(0.3);
    m_launcher.setFeederSpeed(0);
    m_launcher.setHoodAngle(HoodConstants.HoodMaxAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_launcher.setFlywheelSpeed(0);
    m_launcher.setFeederSpeed(0);
    m_launcher.setHoodAngle(HoodConstants.HoodMaxAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
