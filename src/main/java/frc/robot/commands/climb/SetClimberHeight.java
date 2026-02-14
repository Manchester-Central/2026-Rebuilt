// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IClimber;

public class SetClimberHeight extends Command {
  private IClimber m_climber;
  private Distance m_targetHeight;

  public SetClimberHeight(IClimber climber, Distance targetHeight) {
    m_targetHeight = targetHeight;
    m_climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_climber.setHeight(m_targetHeight);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
