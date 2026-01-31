package frc.robot.subsystems.launcher;

import com.chaos131.util.ChaosTalonFx;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.interfaces.IFlywheel;

public class Flywheel implements IFlywheel {
    private ChaosTalonFx m_flywheelMotor = new ChaosTalonFx(LauncherConstants.FlywheelCanId, LauncherConstants.LauncherCanBus);
 
    @Override
    public void setFlywheelSpeed (double speed) {
        m_flywheelMotor.set(speed);
    }

    @Override
    public double getFlywheelSpeed () {
        return m_flywheelMotor.get();
    }
}
