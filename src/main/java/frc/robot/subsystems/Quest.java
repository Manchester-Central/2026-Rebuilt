// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.OptionalInt;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.QuestConstants;
import frc.robot.subsystems.drive.Drive;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * A wrapper for the vendordep version of the QuestNav system
 */
public class Quest extends SubsystemBase {
  /** Creates a new Quest. */
  private QuestNav questNav = new QuestNav();
  private boolean useForOdometry;

  // Transformation from the robot origin to the quest center,
  // defined as the point in the center of the front panel
  // Need to validate what the quest uses as its origin,
  // this may be wrong
  private Transform3d robotToQuest = new Transform3d(
    QuestConstants.RobotToQuestXInches, 
    QuestConstants.RobotToQuestYInches,
    QuestConstants.RobotToQuestZInches,
    QuestConstants.RobotToQuestRotation);
  
  // To keep a reference of the swerve drive for sending pose updates
  private Drive m_swerveDrive;
  private Pose3d robotPose = null;
  private Pose3d questPose = null;
  // Constructor
  public Quest(Drive swerveDrive) {
    m_swerveDrive = swerveDrive;
    setUseForOdometry(true);
  }

  public void resetPose(Pose3d newRobotPose) {
    robotPose = newRobotPose;
    questPose = robotPose.transformBy(robotToQuest);
    questNav.setPose(questPose);
  }

  public void setUseForOdometry(boolean isEnabled) {
    useForOdometry = isEnabled;
    Logger.recordOutput("Quest/useForOdometry", useForOdometry);
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();
    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    OptionalInt battery = questNav.getBatteryPercent();
    Logger.recordOutput("Quest/isConnected", questNav.isConnected());
    Logger.recordOutput("Quest/isTracking", questNav.isTracking());
    Logger.recordOutput("Quest/battery", battery.isPresent() ? battery.getAsInt() : 0);
    // Logger.recordOutput("Quest/isResetActive", isResetActive);

    robotPose = null;
    questPose = null;

    if (poseFrames.length > 0) {
      // Only logs the most recent (if multiple) poses from the quest in the
      // "Quest/robotPose" topic listed below. Bails from the function if we
      // fail to make a robot pose.
      questPose = poseFrames[poseFrames.length - 1].questPose3d();
      robotPose = questPose.transformBy(robotToQuest.inverse());
    }
    if (robotPose == null) {
      return;
    }
    Logger.recordOutput("Quest/questPose", questPose);
    Logger.recordOutput("Quest/robotPose", robotPose);
    
    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
      0.02, // Trust down to 2cm in X direction
      0.02, // Trust down to 2cm in Y direction
      0.035 // Trust down to 2 degrees rotational
    );

    if (questNav.isConnected() && questNav.isTracking() && DriverStation.isEnabled() && useForOdometry) {
      // Loop over the pose data frames and send them to the pose estimator
      for (PoseFrame questFrame : poseFrames) {
        // Get the pose of the Quest
        Pose3d questPose3d = questFrame.questPose3d();
        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        // Transform by the mount pose to get your robot pose
        robotPose = questPose3d.transformBy(robotToQuest.inverse());

        // Add the measurement to our estimator
        m_swerveDrive.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
      }
    } 
  }
}
