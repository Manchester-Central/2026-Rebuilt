package frc.robot.constants;

import com.chaos131.can.CanConstants.CanId;

public final class ArenaConstants {
  public enum MotorIDs {
    Launcher1(0),
    Launcher2(1),
    Indexer(2),
    Intake(3),
    IntakePivot(4);

    public final int canIdx;
    MotorIDs(int id) {
      canIdx = id;
    }
  };

  public static CanId[][] motorCanIDs = new CanId[][]{
    {CanId.ID_20, CanId.ID_21, CanId.ID_22, CanId.ID_23, CanId.ID_24},
    {CanId.ID_25, CanId.ID_26, CanId.ID_27, CanId.ID_28, CanId.ID_29},
    {CanId.ID_30, CanId.ID_31, CanId.ID_32, CanId.ID_33, CanId.ID_34},
    {CanId.ID_35, CanId.ID_36, CanId.ID_37, CanId.ID_38, CanId.ID_39},
    {CanId.ID_40, CanId.ID_41, CanId.ID_42, CanId.ID_43, CanId.ID_44},
    {CanId.ID_45, CanId.ID_46, CanId.ID_47, CanId.ID_48, CanId.ID_49},
  };

  public static int HopperSize = 50;
}
