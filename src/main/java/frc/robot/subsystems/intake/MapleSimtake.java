// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import frc.robot.constants.ArenaConstants;
import frc.robot.constants.IntakeConstants.PivotConstants;
import frc.robot.constants.RobotDimensions;

public class MapleSimtake extends Intake {
  protected SwerveDriveSimulation drive;
  protected IntakeSimulation intakeSim;

  public MapleSimtake(int id, SwerveDriveSimulation drive) {
    super(id);
    this.drive = drive;

    intakeSim = IntakeSimulation.OverTheBumperIntake(
      "Fuel",
      drive,
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

  @Override
  public void periodic() {
    super.periodic();

    if (atTargetAngle(PivotConstants.DeployAngle.get())) {
      intakeSim.startIntake();
    } else {
      intakeSim.stopIntake();
    }
  }
}
