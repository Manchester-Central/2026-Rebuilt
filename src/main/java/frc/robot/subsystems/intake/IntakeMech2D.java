// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IIntake;

public class IntakeMech2D extends SubsystemBase {
  @AutoLogOutput(key = "Intake/IntakeMech2D")
  LoggedMechanism2d m_intakeBase;
  LoggedMechanismRoot2d m_intakeRoot;
  LoggedMechanismLigament2d m_intakeLigament;

  IIntake m_intake;

  /** Creates a new IntakeMech2D. */
  public IntakeMech2D(IIntake intake) {
    m_intakeBase = new LoggedMechanism2d(Inches.of(26), Inches.of(28.5));
    m_intakeRoot = m_intakeBase.getRoot("Intake", 0, 0.2);
    m_intakeLigament = m_intakeRoot.append(new LoggedMechanismLigament2d("IntakeLigament", Inches.of(10), Degrees.of(90)));

    m_intake = intake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_intakeLigament.setAngle(m_intake.getPivotAngle());
  }
}
