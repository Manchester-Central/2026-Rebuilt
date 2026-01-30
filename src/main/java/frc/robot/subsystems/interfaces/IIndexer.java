package frc.robot.subsystems.interfaces;

public interface IIndexer {
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
