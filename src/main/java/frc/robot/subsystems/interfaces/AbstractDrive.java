package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;

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
