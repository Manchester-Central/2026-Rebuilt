// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.vision.CameraSpecs;
import com.chaos131.vision.LimelightCamera;
import com.chaos131.vision.VisionData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Our implementation of a LimeLightCamera.
 */
public class Camera extends LimelightCamera {
  /** Creates a new FrontCamera. */
  public Camera(
      String name,
      LimelightVersion limelightVersion,
      CameraSpecs specs,
      Supplier<Pose2d> poseSupplier,
      Consumer<VisionData> poseConsumer,
      Supplier<Double> robotSpeedSupplier,
      Supplier<Double> robotRotationSpeedSupplier) {
    super(
        name,
        limelightVersion,
        specs,
        poseSupplier,
        poseConsumer,
        robotSpeedSupplier,
        robotRotationSpeedSupplier);
  }

  public Pose3d getTargetPose3dRobotSpace() {
    return LimelightHelpers.getTargetPose3d_RobotSpace(m_name);
  }

  public Pose3d getBotPose3dTargetSpace() {
    return LimelightHelpers.getBotPose3d_TargetSpace(m_name);
  }
}