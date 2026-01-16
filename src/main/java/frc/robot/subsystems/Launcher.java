package frc.robot.subsystems;

import com.chaos131.util.ChaosTalonFx;

import frc.robot.Constants;

public class Launcher {
    private ChaosTalonFx m_launcherMotor = new ChaosTalonFx(Constants.LauncherCanId, Constants.LauncherCanBus);

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
