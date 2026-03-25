package frc.robot.constants;

import com.chaos131.can.CanConstants.CanId;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class ArenaConstants {
  public enum MotorIDs {
    Launcher1(0),
    Launcher2(1),
    FeederTop(2),
    FeederBottom(3),
    IntakeRoller(4),
    IntakePivot(5),
    IntakePivotEncoder(6);

    public final int canIdx;
    MotorIDs(int id) {
      canIdx = id;
    }
  };

  public static final int numAdditionalRobots = 0;
  public static CanId[][] motorCanIDs = new CanId[][] {
    {CanId.ID_20, CanId.ID_21, CanId.ID_22, CanId.ID_23, CanId.ID_24, CanId.ID_25, CanId.ID_26},
    {CanId.ID_27, CanId.ID_28, CanId.ID_29, CanId.ID_30, CanId.ID_31, CanId.ID_32, CanId.ID_33},
    {CanId.ID_34, CanId.ID_35, CanId.ID_36, CanId.ID_37, CanId.ID_38, CanId.ID_39, CanId.ID_40},
    {CanId.ID_41, CanId.ID_42, CanId.ID_43, CanId.ID_44, CanId.ID_45, CanId.ID_46, CanId.ID_47},
    {CanId.ID_48, CanId.ID_49, CanId.ID_50, CanId.ID_51, CanId.ID_52, CanId.ID_53, CanId.ID_54},
    {CanId.ID_55, CanId.ID_56, CanId.ID_57, CanId.ID_58, CanId.ID_58, CanId.ID_59, CanId.ID_60},
  };
  public static Pose2d[] startingPoses = new Pose2d[] {
    // 3 Blue
    new Pose2d(2, 2, Rotation2d.k180deg),
    new Pose2d(2, 4, Rotation2d.k180deg),
    new Pose2d(2, 6, Rotation2d.k180deg),
    // 3 Red
    new Pose2d(14, 2, Rotation2d.kZero),
    new Pose2d(14, 4, Rotation2d.kZero),
    new Pose2d(14, 6, Rotation2d.kZero),
  };

  public static Pose2d[] waitingPoses = new Pose2d[] {
    // 3 Blue
    new Pose2d(-4, 2, Rotation2d.kZero),
    new Pose2d(-4, 4, Rotation2d.kZero),
    new Pose2d(-4, 6, Rotation2d.kZero),
    // 3 Red
    new Pose2d(20, 2, Rotation2d.k180deg),
    new Pose2d(20, 4, Rotation2d.k180deg),
    new Pose2d(20, 6, Rotation2d.k180deg),
  };

  public static int HopperSize = 50;
}
