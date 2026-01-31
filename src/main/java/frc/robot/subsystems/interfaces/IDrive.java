package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public interface IDrive {
    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds);
    /** The robot relative velocity in vector form, units are in meters per second. */
    public Translation2d getVelocityVector();
    /** The speed of the robot, based on the robot relative velocity vector. */
    public LinearVelocity getSpeed();
    /** Returns the current rotational velocity of the robot. */
    public AngularVelocity getRotationalSpeed();
    /** Stops the drive. */
    public void stop();
    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX();
    /** Returns the current odometry pose. */
    public Pose2d getPose();
    /** Returns the current rotation of the robot. */
    public Rotation2d getRotation();
    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose);
    /** Updates the SwervePoseEstimator with noisy vision data. */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec();
    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec();
}
