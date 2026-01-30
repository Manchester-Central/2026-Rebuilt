package frc.robot.subsystems.launcher;

import com.chaos131.util.ChaosTalonFx;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.interfaces.IIndexer;

public class Indexer implements IIndexer {
    private ChaosTalonFx m_indexerMotor = new ChaosTalonFx(LauncherConstants.IndexerCanId, LauncherConstants.LauncherCanBus);

    @Override
    public void setIndexerSpeed (double speed) {
        m_indexerMotor.set(speed);
    }

    @Override
    public double getIndexerSpeed () {
        return m_indexerMotor.get();
    }
}
