// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.IndexerConstants;
import frc.robot.subsystems.interfaces.ISimpleLauncher;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SimpleLauncherDefaultCommand extends Command {
  /** Creates a new SimpleLauncherDefaultCommand. */
  private ISimpleLauncher m_launcher;
  private BooleanSupplier m_isManualMode;
  private BooleanSupplier m_isRunLauncher;
  private BooleanSupplier m_isRunIndexer;
  private BooleanSupplier m_isRunUnjam;

  public SimpleLauncherDefaultCommand(ISimpleLauncher launcher, BooleanSupplier isManualMode, BooleanSupplier isRunLauncher, BooleanSupplier isRunFeeder, BooleanSupplier isRunUnjam) {
    // Use addRequirements() here to declare subsystem dependencies.=
    m_launcher = launcher;
    m_isManualMode = isManualMode;
    m_isRunLauncher = isRunLauncher;
    m_isRunIndexer = isRunFeeder;
    m_isRunUnjam = isRunUnjam;

    addRequirements(m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_isManualMode.getAsBoolean()) {
      if (m_isRunLauncher.getAsBoolean()) {
        m_launcher.setFlywheelSpeed(LauncherConstants.LauncherSpeed.get()); // TODO: add dashboard number
      } else {
        m_launcher.setFlywheelSpeed(0);
      }
      if (m_isRunIndexer.getAsBoolean()) {
        m_launcher.setIndexerSpeed(IndexerConstants.IndexerSpeed.get()); // TODO: add dashboard number
      } else if (m_isRunUnjam.getAsBoolean()) {
        m_launcher.setIndexerSpeed(IndexerConstants.UnjamSpeed.get());
      } else {
        m_launcher.setIndexerSpeed(0);
      }
      
      return;
    }
    m_launcher.setFlywheelSpeed(0);
    m_launcher.setIndexerSpeed(0);
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
