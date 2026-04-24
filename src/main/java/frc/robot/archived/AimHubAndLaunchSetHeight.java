// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.archived;

// import java.util.Optional;

// import com.chaos131.poses.FieldPose;
// import com.chaos131.poses.FieldPose2026;

// import edu.wpi.first.units.measure.Distance;
// import frc.robot.commands.launcher.BaseLaunchCommand;
// import frc.robot.constants.FieldDimensions;
// import frc.robot.constants.LauncherConstants.FeederConstants;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.launcher.Launcher;

// /**
//  * Creates a launch command using physics w/ a moving hood
//  */
// public class AimHubAndLaunchSetHeight extends BaseLaunchCommand {

//   public AimHubAndLaunchSetHeight(Launcher launcher, Drive swerveDrive, Intake intake) {
//     super(launcher, swerveDrive, intake);
//   }

//   @Override
//   protected Optional<FieldPose> getTargetPose() {
//     return Optional.of(FieldPose2026.HubCenter);
//   }

//   @Override
//   protected Optional<Distance> getTargetHeight() {
//     return Optional.of(FieldDimensions.HubHeight);
//   }

//   @Override
//   protected void prepLauncher() {
//     m_launcher.setTargets(
//       m_launcher.getVelocityForTargetSetHeight(
//         m_swerveDrive, 
//         FieldPose2026.HubCenter.getCurrentAlliancePose(), 
//         FieldDimensions.HubHeight).times(m_launcher.getLossFactor()), 
//       m_launcher.getPitchForTarget(
//         m_swerveDrive, 
//         FieldPose2026.HubCenter.getCurrentAlliancePose(), 
//         FieldDimensions.HubHeight));
//   }

//   @Override
//   protected void enableFeederForLauncher() {
//     m_launcher.setFeederSpeed(FeederConstants.FeederSpeed.get());
//   }

//   @Override
//   protected boolean isLauncherReady() {
//     return m_launcher.atTargets();
//   }
// }
