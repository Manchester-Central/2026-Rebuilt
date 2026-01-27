package frc.robot.subsystems.launcher;

import com.chaos131.util.ChaosTalonFx;

import frc.robot.Constants.LauncherConstants;

public class Flywheel {
    private ChaosTalonFx m_flywheelMotor = new ChaosTalonFx(LauncherConstants.FlywheelCanId, LauncherConstants.LauncherCanBus);
 
    /**
     * Sets the speed of the flywheel between -1 and 1.
     */
    public void setFlywheelSpeed (double speed) {
        m_flywheelMotor.set(speed);
    }

    /**
    * Returns the flywheel speed.
    */
    public double getFlywheelSpeed () {
        return m_flywheelMotor.get();
    }
}
