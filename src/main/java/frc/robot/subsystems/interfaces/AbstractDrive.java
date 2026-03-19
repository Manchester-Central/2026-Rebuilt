package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.DriveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;

public abstract class AbstractDrive extends SubsystemBase {
    public static final double DRIVE_BASE_RADIUS =
        Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    protected static final double ROBOT_MASS_KG = 74.088;
    protected static final double ROBOT_MOI = 6.883;
    protected static final double WHEEL_COF = 1.2;

    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          DriveConstants.RobotMass,
          DriveConstants.RobotMoi,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              DriveConstants.WheelCof,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

    public void setupAutoBuilder() {
        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::runVelocity,
            new PPHolonomicDriveController(
                DriveConstants.TranslationalControlPIDConstants, DriveConstants.RotationalControlPIDConstants),
            PP_CONFIG,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });
    }

    public abstract double[] getWheelRadiusCharacterizationPositions();
    public abstract double getFFCharacterizationVelocity();

    public abstract void addVisionMeasurement(Pose2d pose, double time, Matrix<N3, N1> devs);
    public abstract Pose2d getPose();
    public abstract ChassisSpeeds getChassisSpeeds();
    public abstract LinearVelocity getSpeed();
    public abstract Rotation2d getRotation();
    public abstract Translation2d getVelocityVector();
    public abstract AngularVelocity getRotationalSpeed();

    public abstract void stop();
    public abstract void stopWithX();
    public abstract void setPose(Pose2d pose);
    public abstract void runVelocity(ChassisSpeeds speeds);

    public abstract Command sysIdQuasistatic(Direction kforward);
    public abstract Command sysIdDynamic(Direction kreverse);
    public abstract void runCharacterization(double output);
}
