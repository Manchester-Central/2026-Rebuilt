package frc.robot.subsystems;

import com.chaos131.util.ChaosTalonFx;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
    private ChaosTalonFx m_launcherMotor = new ChaosTalonFx(LauncherConstants.LauncherCanId, LauncherConstants.LauncherCanBus);

    public Launcher () {} 

    /**
     * Sets the speed of the launcher between -1 and 1.
     */
    public void setLauncherSpeed (double speed) {
        m_launcherMotor.setSpeed(speed);
    }

    /**
    * Returns the launcher speed.
    */
    public double getLauncherSpeed () {
        return m_launcherMotor.get();
    }
}
