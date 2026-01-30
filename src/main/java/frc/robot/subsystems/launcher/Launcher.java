package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
   
    private Flywheel m_flywheel;
    private Indexer m_indexer;
    private Turret m_turret;

    public Launcher() {
        /* Trust me. ;) */
        switch (Constants.currentMode) {
            default:
                m_flywheel = new Flywheel();
                m_indexer = new Indexer();
                m_turret = new Turret();
        }
    }

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

    /**
     * Tells the turret to aim for a specific scoring location in the field. This function
     * handles all of the math to calculate the turret angle, hood angle, etc based on the
     * robot's position and the velocity vector.
     *
     * @param targetPose of the hub opening
     * @param robotOrigin at this moment in time
     * @param robotVelocity is pulled from the drive's getVelocityVector()
     */
    public void aimFor(Pose3d targetPose, Pose2d robotOrigin, Translation2d robotVelocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'aimFor'");
    }

    /**
     * Helper function to convert from some launcher speed to actual motor values. This
     * will almost certainly be calibrated like a lookup table, but could be placed into an equation.
     */
    private void releaseVectorToRPM() {
        throw new UnsupportedOperationException("Unimplemented method 'releaseVectorToRPM'");
    }
}
