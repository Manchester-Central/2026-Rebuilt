package frc.robot.subsystems.launcher;

import com.chaos131.util.ChaosCanCoder;
import com.chaos131.util.ChaosTalonFx;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.LauncherConstants;

public class Turret implements Iturret {
    private ChaosTalonFx m_turretMotor = new ChaosTalonFx(LauncherConstants.TurretCanId, LauncherConstants.LauncherCanBus);
    private ChaosCanCoder m_turretCanCoder = new ChaosCanCoder(LauncherConstants.TurretCoderId, LauncherConstants.LauncherCanBus);

    public boolean setTurretSpeed(double speed) {
        m_turretMotor.set(speed);
        return true;
    }

    public double getTurretSpeed() {
        return m_turretMotor.get();
    }

    @Override
    public boolean setTargetPosition(Angle targetAngle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetPosition'");
    }
    
}
