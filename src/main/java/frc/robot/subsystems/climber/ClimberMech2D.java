// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IClimber;

public class ClimberMech2D extends SubsystemBase {
  @AutoLogOutput(key = "Climber/ClimberMech2D")
  LoggedMechanism2d m_climberBase;
  LoggedMechanismRoot2d m_climberRoot;
  LoggedMechanismLigament2d m_climberLigament;

  IClimber m_climber;

  /** Creates a new ClimberMech2D. */
  public ClimberMech2D(IClimber climber) {
    m_climberBase = new LoggedMechanism2d(Inches.of(26), Inches.of(28.5));
    m_climberRoot = m_climberBase.getRoot("Climber", 0, 0.4);
    m_climberLigament = m_climberRoot.append(new LoggedMechanismLigament2d("ClimberLigament", Inches.of(0.001), Degrees.of(90)));

    m_climber = climber;
  }

  @Override
  public void periodic() {
    m_climberLigament.setLength(m_climber.getHeight());
  }
}
