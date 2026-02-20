// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.vision.CameraSpecs;
import com.chaos131.vision.LimelightCamera;
import com.chaos131.vision.VisionData;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
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

  public Pose3d getBotPose3d() {
    var botPoseArray = m_botpose.getDoubleArray(new double[0]);
    if (botPoseArray.length > 6) {
      return null;
    }
    return LimelightHelpers.toPose3D(botPoseArray);
  }

  public Pose3d getTargetPose3dRobotSpace() {
    return LimelightHelpers.getTargetPose3d_RobotSpace(m_name);
  }

  public Pose3d getBotPose3dTargetSpace() {
    return LimelightHelpers.getBotPose3d_TargetSpace(m_name);
  }

  protected double[] calculateTranslationalDeviations(double distance, double tagCount) {
    double[] basic_deviations = super.calculateTranslationalDeviations(distance, tagCount);
    return new double[] {
      basic_deviations[0] * Timer.getFPGATimestamp(),
      basic_deviations[1] * Timer.getFPGATimestamp(),
      basic_deviations[2] * Timer.getFPGATimestamp() 
    };
  }

  @Override
  public double calculateConfidence(Pose3d pose, int tagCount, double distance, double deviation) {
    if (pose.equals(new Pose3d())) {
      return 0;   
    }
    return super.calculateConfidence(pose, tagCount, distance, deviation);
  }

}