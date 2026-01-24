package frc.robot.subsystems.launcher;

import com.chaos131.util.ChaosTalonFx;

import frc.robot.Constants.LauncherConstants;

public class Turret {
    private ChaosTalonFx m_turretMotor = new ChaosTalonFx(LauncherConstants.TurretCanId, LauncherConstants.LauncherCanBus);

    public void setTurretSpeed(double speed) {
        m_turretMotor.set(speed);
    }

    public double getTurretSpeed() {
        return m_turretMotor.get();
    }
    
}
