package frc.robot.subsystems.launcher;

import com.chaos131.util.ChaosTalonFx;

import frc.robot.Constants.LauncherConstants;

public class Indexer {
    private ChaosTalonFx m_indexerMotor = new ChaosTalonFx(LauncherConstants.IndexerCanId, LauncherConstants.LauncherCanBus);

    public void setIndexerSpeed (double speed) {
        m_indexerMotor.set(speed);
    }

    public double getIndexerSpeed () {
        return m_indexerMotor.get();
    }
}
