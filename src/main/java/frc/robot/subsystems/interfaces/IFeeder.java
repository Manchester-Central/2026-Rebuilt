package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IFeeder extends Subsystem {
    /**
     * Sets the speed of the feeder
     * @param speed double in the range of [-1.0, 1.0]
     */
    public void setFeederSpeed(double speed);

    /**
     * @return the current speed of the feeder, in the range of [-1.0, 1.0]
     */
    public double getFeederSpeed();
}
