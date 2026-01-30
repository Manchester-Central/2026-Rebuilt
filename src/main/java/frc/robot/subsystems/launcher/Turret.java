package frc.robot.subsystems.launcher;

import com.chaos131.util.ChaosCanCoder;
import com.chaos131.util.ChaosTalonFx;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.interfaces.ITurret;

public class Turret implements ITurret {
    private ChaosTalonFx m_turretMotor = new ChaosTalonFx(LauncherConstants.TurretCanId, LauncherConstants.LauncherCanBus);
    private ChaosCanCoder m_turretCanCoder = new ChaosCanCoder(LauncherConstants.TurretCoderId, LauncherConstants.LauncherCanBus);

    public void setTurretSpeed(double speed) {
        m_turretMotor.set(speed);
    }

    public double getTurretSpeed() {
        return m_turretMotor.get();
    }

    @Override
    public boolean setTargetPosition(Angle targetAngle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetPosition'");
    }
    
    public void aimFieldOriented(Pose2d robotPose, Angle toFace) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'aimFieldOriented'");
    }

    @Override
    public void aimFieldOriented(Pose2d robotPose, Pose2d target) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'aimFieldOriented'");
    }
}
