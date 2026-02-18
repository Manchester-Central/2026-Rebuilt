// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.chaos131.util.Color;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ILauncher;

public class LauncherMech2D extends SubsystemBase {
  @AutoLogOutput(key = "Launcher/LauncherMech2D")
  LoggedMechanism2d m_launcherBase;
  LoggedMechanismRoot2d m_launcherRoot;
  LoggedMechanismLigament2d m_feederLigament;
  LoggedMechanismLigament2d m_launcherLigament;

  ILauncher m_launcher;

  /** Creates a new IntakeMech2D. */
  public LauncherMech2D(ILauncher launcher) {
    m_launcherBase = new LoggedMechanism2d(Inches.of(26), Inches.of(28.5));
    m_launcherRoot = m_launcherBase.getRoot("Launcher", 0.7, 0.4);
    m_feederLigament = m_launcherRoot.append(new LoggedMechanismLigament2d("FeederLigament", Inches.of(8), Degrees.of(75)));
    m_launcherLigament = m_feederLigament.append(new LoggedMechanismLigament2d("LauncherLigament", Inches.of(6), Degrees.of(-20)));

    m_launcher = launcher;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_feederLigament.setColor(Color.fromDutyCycle(m_launcher.getFeederSpeed()));
    m_launcherLigament.setColor(Color.fromDutyCycle(m_launcher.getFlywheelSpeed()));
  }
}
