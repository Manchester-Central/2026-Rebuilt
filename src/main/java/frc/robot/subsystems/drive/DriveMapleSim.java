package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class DriveMapleSim extends AbstractDrive {
  private DriveTrainSimulationConfig m_swerveConfig;
  private SwerveDriveSimulation m_simSwerve;

  public DriveMapleSim(Pose2d initialPose) {
    m_simSwerve = new SwerveDriveSimulation(m_swerveConfig, getPose());
  }

  @Override
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    // TODO: Needed?
    // ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    // SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);
    m_simSwerve.setRobotSpeeds(speeds);
  }

  @Override
  public void runCharacterization(double output) {
    // No-op with maple sim, leave the function signature so the rest of the code is cleaner though
    return;
  }

  @Override
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  @Override
  public void stopWithX() {
    // Maybe add a lot of mass?
    m_simSwerve.setRobotSpeeds(new ChassisSpeeds());
  }

  @Override
  public Command sysIdQuasistatic(Direction direction) {
    // No-op with maple sim, leave the function signature so the rest of the code is cleaner though
    return new InstantCommand();
  }

  @Override
  public Command sysIdDynamic(Direction direction) {
    // No-op with maple sim, leave the function signature so the rest of the code is cleaner though
    return new InstantCommand();
  }

  @Override
  public double[] getWheelRadiusCharacterizationPositions() {
    // No-op with maple sim, leave the function signature so the rest of the code is cleaner though
    return new double[]{0,0,0,0};
  }

  @Override
  public double getFFCharacterizationVelocity() {
    // No-op with maple sim, leave the function signature so the rest of the code is cleaner though
    return 0.0;
  }

  @Override
  public Pose2d getPose() {
    return m_simSwerve.getSimulatedDriveTrainPose();
  }

  @Override
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @Override
  public void setPose(Pose2d pose) {
    m_simSwerve.setSimulationWorldPose(pose);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // No-op with maple sim, leave the function signature so the rest of the code is cleaner though
    return;
  }
}
