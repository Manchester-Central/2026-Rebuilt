// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeManualCommand extends Command {
  private IIntake m_intake;
  
  /** Creates a new IntakeDefaultCommand. */
  public IntakeManualCommand(IIntake intake) {
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
    // TODO: determine logic while in manual mode
    m_intake.setRollerSpeed(0);
    m_intake.setPivotSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setRollerSpeed(0);
    m_intake.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
