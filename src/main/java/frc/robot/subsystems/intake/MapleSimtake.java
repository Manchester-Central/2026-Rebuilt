// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

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
    // parent version of periodic just calls Logger, so we can ignore it for our own
    // super.periodic();

    Logger.recordOutput("Robot"+m_id+"/IntakeTargetAngle", m_targetAngle.in(Degrees));
    Logger.recordOutput("Robot"+m_id+"/IntakeCurrentAngle", getPivotAngle().in(Degrees));
 
    if (atTargetAngle(PivotConstants.DeployAngle.get()) && getRollerSpeed() > 0.1) {
      intakeSim.startIntake();
    } else {
      intakeSim.stopIntake();
    }
  }
}
