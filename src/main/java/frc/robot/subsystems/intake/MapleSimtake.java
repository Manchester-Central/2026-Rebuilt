// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import frc.robot.constants.ArenaConstants;
import frc.robot.constants.RobotDimensions;
import frc.robot.subsystems.drive.DriveMapleSim;

public class MapleSimtake extends Intake {
  protected DriveMapleSim drive;
  protected IntakeSimulation intakeSim;

  public MapleSimtake(int id, DriveMapleSim drive) {
    super(id);
    this.drive = drive;

    intakeSim = IntakeSimulation.OverTheBumperIntake(
      "Fuel",
      drive.sim,
      RobotDimensions.FrameWidth,
      RobotDimensions.IntakeRange,
      IntakeSide.BACK,
      ArenaConstants.HopperSize);
  }

  @Override
  public int getNumGamePieces() {
    return intakeSim.getGamePiecesAmount();
  }

  @Override
  public boolean claimGamePiece() {
    return intakeSim.obtainGamePieceFromIntake();
  }
}
