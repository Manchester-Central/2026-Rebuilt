package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IIndexer extends Subsystem {
    /**
     * Sets the speed of the indexer
     * @param speed double in the range of [-1.0, 1.0]
     */
    public void setIndexerSpeed(double speed);

    /**
     * @return the current speed of the indexer, in the range of [-1.0, 1.0]
     */
    public double getIndexerSpeed();
}
