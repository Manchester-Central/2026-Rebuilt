package frc.robot.subsystems.launcher;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import frc.robot.constants.ArenaConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.ArenaConstants.MotorIDs;
import frc.robot.constants.LauncherConstants.IndexerConstants;
import frc.robot.subsystems.interfaces.IIndexer;

public class Indexer implements IIndexer {
    private ChaosTalonFx m_indexerMotor;

    @SuppressWarnings("unused")
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("Launcher/Indexer/Indexer Motor", m_indexerMotor).withCurrentLimits();

    public Indexer(int id) {
        if (id == 0) {
            m_indexerMotor = new ChaosTalonFx(IndexerConstants.IndexerCanId, LauncherConstants.LauncherCanBus, IndexerConstants.Config);
        } else {
            m_indexerMotor = new ChaosTalonFx(ArenaConstants.motorCanIDs[id][MotorIDs.Indexer.canIdx], LauncherConstants.LauncherCanBus, IndexerConstants.Config);
        }
    }

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
