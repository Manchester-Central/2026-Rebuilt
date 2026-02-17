package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.MultiplayerSim.MultiplayerArena;
import frc.robot.subsystems.interfaces.AbstractDrive;

public class DriveMapleSim extends AbstractDrive {
  private DriveTrainSimulationConfig m_swerveConfig = DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(54))
        .withCustomModuleTranslations(getModuleTranslations())
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(new SwerveModuleSimulationConfig(
                          DCMotor.getKrakenX60(1),
                          DCMotor.getKrakenX60(1),
                          TunerConstants.FrontLeft.DriveMotorGearRatio,
                          TunerConstants.FrontLeft.SteerMotorGearRatio,
                          Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                          Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                          Meters.of(TunerConstants.FrontLeft.WheelRadius),
                          KilogramSquareMeters.of(0.005), //KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                          1.2));
  public SwerveDriveSimulation sim;

  public DriveMapleSim(Pose2d initialPose) {
    sim = new SwerveDriveSimulation(m_swerveConfig, initialPose);
    // SimulatedArena.getInstance().addDriveTrainSimulation(sim);
    MultiplayerArena.Instance.addDriveTrainSimulation(sim);
  }

  @Override
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    // TODO: Needed?
    // ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    // SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);
    Logger.recordOutput("Robot0/ChassisSpeeds", speeds);
    sim.setRobotSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation()));
  }

  @Override
  @AutoLogOutput(key = "Robot0/getChassisSpeeds")
  public ChassisSpeeds getChassisSpeeds() {
    // return sim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    return sim.getDriveTrainSimulatedChassisSpeedsRobotRelative();
  }

  @Override
  public void runCharacterization(double output) {
    // No-op with maple sim, leave the function signature so the rest of the code is cleaner though
    return;
  }

  @Override
  public void stop() {
    sim.setRobotSpeeds(new ChassisSpeeds());
  }

  @Override
  public void stopWithX() {
    // Maybe add a lot of mass?
    sim.setRobotSpeeds(new ChassisSpeeds());
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
    return sim.getSimulatedDriveTrainPose();
  }

  @Override
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @Override
  public void setPose(Pose2d pose) {
    sim.setSimulationWorldPose(pose);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // No-op with maple sim, leave the function signature so the rest of the code is cleaner though
    return;
  }

  /**
   * Returns the velocity vector of the drive train,
   * units come from getVelocity3dMPS in MapleSim.
   */
  @Override
  public Translation2d getVelocityVector() {
    Vector2 lv = sim.getLinearVelocity();
    return new Translation2d(lv.x, lv.y);
  }

  /**
   * Returns the LinearSpeed unit form of getVelocityVector().
   */
  @Override
  public LinearVelocity getSpeed() {
    return MetersPerSecond.of(getVelocityVector().getNorm());
  }

  /**
   * Returns the angular velocity of the simulated drive train,
   * units come from omegaRadiansPerSecond in MapleSim.
   */
  @Override
  public AngularVelocity getRotationalSpeed() {
    var av = sim.getAngularVelocity();
    return RadiansPerSecond.of(av);
  }
}
