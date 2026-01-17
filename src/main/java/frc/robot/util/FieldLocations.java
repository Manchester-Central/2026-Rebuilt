package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class FieldLocations {
  public static final Translation3d BlueHubCenter = new Translation3d(Inches.of(181.56),Inches.of(158.32), Inches.of(72));
  public static final Translation3d RedHubCenter = new Translation3d(Inches.of(468.56),Inches.of(158.32), Inches.of(72));
  public static final Translation3d BlueAllianceBallBox = new Translation3d(Inches.of(27.0/2), Inches.of(234.32), Inches.of(0));
  public static final Translation3d RedAllianceBallBox = new Translation3d(Inches.of(27.0/2), Inches.of(234.32), Inches.of(0));

  public static AprilTagFields aprilTagField = AprilTagFields.k2026RebuiltAndymark;
  public static AprilTagFieldLayout aprilTagLayout;
  public static FieldPose2026 Outpost;
  public static FieldPose2026 HubCenter;
  public static FieldPose2026 AllianceBallBox;

  public static void initialize() throws IOException {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
    }
    aprilTagLayout = new AprilTagFieldLayout(aprilTagField.toString());
    Translation2d midpoint = new Translation2d(aprilTagLayout.getFieldLength()/2.0, aprilTagLayout.getFieldWidth()/2.0);
    HubCenter = new FieldPose2026(midpoint, Alliance.Blue, "HubCenter", new Pose3d(BlueHubCenter, new Rotation3d()));
    Outpost = new FieldPose2026(midpoint, Alliance.Blue, "Outpost", aprilTagLayout.getTagPose(29).get());
    AllianceBallBox = new FieldPose2026(midpoint, Alliance.Blue, "AllianceBallBox", new Pose3d(BlueAllianceBallBox, new Rotation3d()));
  }
}
