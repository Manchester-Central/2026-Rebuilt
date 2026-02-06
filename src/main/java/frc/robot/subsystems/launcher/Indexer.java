package frc.robot.subsystems.launcher;

import com.chaos131.ctre.ChaosTalonFx;

import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.IndexerConstants;
import frc.robot.subsystems.interfaces.IIndexer;

public class Indexer implements IIndexer {
    private ChaosTalonFx m_indexerMotor = new ChaosTalonFx(IndexerConstants.IndexerCanId, LauncherConstants.LauncherCanBus, IndexerConstants.Config);

    public Indexer() {
        m_indexerMotor.applyConfig();
    }

    @Override
    public void setIndexerSpeed (double speed) {
        m_indexerMotor.set(speed);
    }

    @Override
    public double getIndexerSpeed () {
        return m_indexerMotor.get();
    }
}
