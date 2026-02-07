package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public interface IFlywheel {
    /**
     * Sets the speed of the flywheel
     * @param speed double in the range of [-1.0, 1.0]
     */
    public void setFlywheelSpeed(double speed);

    /**
     * @return the current speed of the flywheel, in the range of [-1.0, 1.0]
     */
    public double getFlywheelSpeed();

    /**
     * Sets the target velocity of the flywheel 
     * @param angularVelocity the desired angular velocity
     */
    public void setFlywheelVelocity(AngularVelocity angularVelocity);

    /**
     * Sets the target velocity of the flywheel 
     * @param linearVelocity the desired linear velocity
     */
    public void setFlywheelVelocity(LinearVelocity linearVelocity);

    /**
     * Returns the linear velocity of the left side motor
     * @return that linear velocity
     */
    public LinearVelocity getLeftLinearVelocity();

    /**
     * Checks if the specific motor is at the target linear velocity
     * @return true if within tolerance
     */
    public boolean atTargetLeft();

    /**
     * Returns the linear velocity of the left side motor
     * @return that linear velocity
     */
    public LinearVelocity getRightLinearVelocity();

    /**
     * Checks if the specific motor is at the target linear velocity
     * @return true if within tolerance
     */
    public boolean atTargetRight();
}
