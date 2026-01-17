package frc.robot.util;

import com.chaos131.poses.PivotedFieldPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldPose2026 extends PivotedFieldPose {
  public FieldPose2026(Translation2d midpoint, Alliance defaultAlliance, String name, Pose3d defaultPose) {
    super(midpoint, defaultAlliance, name, defaultPose);
  }
}
