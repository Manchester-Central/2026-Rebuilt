// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeDefaultCommand extends Command {
 private Intake m_intake;
 private BooleanSupplier m_isManualMode;
 private BooleanSupplier m_isRunIntake;
 private DoubleSupplier m_intakePivotSpeed;
  /** Creates a new IntakeDefaultCommand. */
  public IntakeDefaultCommand(Intake intake, BooleanSupplier isManualMode, BooleanSupplier isRunIntake, DoubleSupplier intakePivotSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_isManualMode = isManualMode;
    m_isRunIntake = isRunIntake;
    m_intakePivotSpeed = intakePivotSpeed;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_isManualMode.getAsBoolean()) {
      if (m_isRunIntake.getAsBoolean()) {
        m_intake.setRollerSpeed(0.1);
      } else {
        m_intake.setRollerSpeed(0);
      }
      m_intake.setPivotSpeed(m_intakePivotSpeed.getAsDouble() * IntakeConstants.ManualPivotSpeedMultiplier.get());
      return;
    }
    m_intake.setRollerSpeed(0);
    m_intake.setPivotSpeed(0);
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
