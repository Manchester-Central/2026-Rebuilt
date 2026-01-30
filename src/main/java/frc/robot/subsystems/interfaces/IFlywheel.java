package frc.robot.subsystems.interfaces;

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
}
