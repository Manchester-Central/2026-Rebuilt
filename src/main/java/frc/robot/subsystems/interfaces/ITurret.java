package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;

public interface ITurret {
    /**
     * Sets the speed of the turret
     * @param speed double in the range of [-1.0, 1.0]
     */
    public void setTurretSpeed(double speed);
    /**
     * @return the current speed of the indexer, in the range of [-1.0, 1.0]
     */
    public double getTurretSpeed();

    /**
     * Tells the turret to face a specific field-oriented direction,
     * accounting for the robot's direction. This can't account for
     * any positional difference between the robot origin and the
     * turret position.
     * 
     * <p> NOTE: This should differ from aimFieldOriented(Pose2d, Pose2d)
     * 
     * @param robotPose Pose2d of the robot center
     * @param toFace Angle to face, from the center of the robot
     */
    public void aimFieldOriented(Pose2d robotPose, Angle toFace);

    /**
     * Tells the turret to face a specific point, <b>accounting</b> for
     * the turret's position relative to the robot origin. This does not
     * account for the robot's velocity vector.
     * 
     * <p>NOTE: This should differ from aimFieldOriented(Pose2d, Angle)
     * 
     * @param robotPose Pose2d of the robot center
     * @param target to aim at, from the source location
     */
    public void aimFieldOriented(Pose2d robotPose, Pose2d target);
}
