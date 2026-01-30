package frc.robot.subsystems.launcher;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
   
    private Flywheel m_flywheel = new Flywheel();
    private Indexer m_indexer = new Indexer();
    private Turret m_turret = new Turret();

    public Launcher () {} 

    public double getFlywheelSpeed () {
        return m_flywheel.getFlywheelSpeed();
    }
    public double getIndexerSpeed () {
        return m_indexer.getIndexerSpeed();
    }
    public double getTurretSpeed () {
        return m_turret.getTurretSpeed();
    }

    public void setFlywheelSpeed (double speed) {
        m_flywheel.setFlywheelSpeed(speed);
    }
    public void setIndexerSpeed (double speed) {
        m_indexer.setIndexerSpeed (speed);
    }
    public void setTurretSpeed (double speed) {
        m_turret.setTurretSpeed (speed);
    }

    public ArrayList<Pose3d> generateMech3d() {
        return new ArrayList<>();
    }
}
