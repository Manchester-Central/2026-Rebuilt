// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;
import com.chaos131.util.DriveDirection;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.FieldDimensions;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.constants.LauncherConstants.HoodConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.TableRow;

/**
 * Creates a launch command using the lookup table
 */
public class PassLaunch extends BaseLaunchCommand {
  private TableRow m_flywheelTableRow = new TableRow(Inches.of(0), MetersPerSecond.of(0), 0.0);

  private Timer m_intakeTimer = new Timer();

  public PassLaunch(Launcher launcher, Drive swerveDrive, Intake intake) {
    super(launcher, swerveDrive, intake);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_intakeTimer.restart();
  }


  @Override
  protected Optional<FieldPose> getTargetPose() {
    return Optional.of(FieldPose2026.HubCenter);
  }

  @Override
  protected Optional<Distance> getTargetHeight() {
    return Optional.of(FieldDimensions.HubHeight);
  }

  @Override
  protected void preExecute() {
    m_flywheelTableRow = m_launcher.getLaunchLookupTableRow();
  }

  @Override
  protected void prepLauncher() {
    m_launcher.setFlywheelVelocity(m_flywheelTableRow.getLaunchSpeed());
    m_launcher.setHoodAngle(HoodConstants.HoodMinAngle);
   }

  @Override
  protected void enableFeederForLauncher() {
    m_launcher.setFeederSpeed(FeederConstants.BottomFeederSpeed.get(), FeederConstants.TopFeederSpeed.get()); 
  }

  @Override
  protected boolean isFacingLaunchTarget() {
    Angle targetAngle =  DriveDirection.Towards.getAllianceAngle().getMeasure();
    Angle currentAngle = m_swerveDrive.getRotation().getMeasure();

    Logger.recordOutput("Launcher/TargetAngle", targetAngle.in(Degrees));
    Logger.recordOutput("Launcher/CurrentAngle", currentAngle.in(Degrees));
    Logger.recordOutput("Launcher/AngleDif", targetAngle.minus(currentAngle).in(Degrees));

    // TODO Auto-generated method stub
    return targetAngle.minus(currentAngle).lt(LauncherConstants.LaunchYawTolerance.get());
  }

  @Override
  protected boolean isFacingTarget() {
    Angle targetAngle =  DriveDirection.Towards.getAllianceAngle().getMeasure();
    Angle currentAngle = m_swerveDrive.getRotation().getMeasure();

    Logger.recordOutput("Launcher/TargetAngle", targetAngle.in(Degrees));
    Logger.recordOutput("Launcher/CurrentAngle", currentAngle.in(Degrees));
    Logger.recordOutput("Launcher/AngleDif", targetAngle.minus(currentAngle).in(Degrees));

    // TODO Auto-generated method stub
    return targetAngle.minus(currentAngle).lt(LauncherConstants.AimYawTolerance.get());
  }

  // @Override
  // public Angle getIntakePivotAngle() {
  //  double seconds = m_intakeTimer.get();
  //  if(seconds > LauncherConstants.JostleDelay.get().in(Seconds)){
  //   return Degrees.of(Math.max(
  //     PivotConstants.DeployAngle.get().in(Degrees) - (seconds - LauncherConstants.JostleDelay.get().in(Seconds)) * PivotConstants.JostleSpeed.get(), 
  //     LauncherConstants.IntakePivotJostleAngle.get().in(Degrees)));
  //   } else {
  //     return PivotConstants.DeployAngle.get();
  //   }
  // }

  @Override
  public Angle getIntakePivotAngle(){
   double seconds = m_intakeTimer.get();
   int newstep = (int)seconds;
   if(newstep% 2 == 0){
    return IntakeConstants.PivotConstants.DeployAngle.get();
    } else {
      return LauncherConstants.IntakePivotJostleAngle.get();
    }
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    m_launcher.setHoodAngle(HoodConstants.HoodMaxAngle);
  }
}
