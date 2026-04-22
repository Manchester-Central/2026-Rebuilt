// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;

/**
 * Creates a launch command using the lookup table
 */
public class HubLaunchNoJostle extends HubLaunch {

  public HubLaunchNoJostle(Launcher launcher, Drive swerveDrive, Intake intake) {
    super(launcher, swerveDrive, intake);
  }

  @Override
  public Angle getIntakePivotAngle(){
    return IntakeConstants.PivotConstants.DeployAngle.get();
  }

}
