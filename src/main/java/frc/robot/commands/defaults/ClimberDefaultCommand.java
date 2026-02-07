// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.interfaces.IClimber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberDefaultCommand extends Command {
  IClimber m_climber;
  DoubleSupplier m_climberSpeed;
  BooleanSupplier m_isManualMode;

  /** Creates a new ClimberDefaultCommand. */
  public ClimberDefaultCommand(IClimber climber, DoubleSupplier climberSpeed, BooleanSupplier isManualMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_climberSpeed = climberSpeed;
    m_isManualMode = isManualMode;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_isManualMode.getAsBoolean()) {
      m_climber.setClimberSpeed(m_climberSpeed.getAsDouble() * ClimberConstants.ManualSpeedMultiplier.get());
    } else {
      m_climber.setClimberSpeed(0); 
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
