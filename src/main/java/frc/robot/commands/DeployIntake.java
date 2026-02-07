// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.PivotConstants;
import frc.robot.subsystems.interfaces.IIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployIntake extends Command {
  IIntake m_intake;
  /** Creates a new DeployIntake. */
  public DeployIntake(IIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPivotAngle(PivotConstants.DeployAngle);
    m_intake.setRollerSpeed(IntakeConstants.RollerSpeed.get());
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
