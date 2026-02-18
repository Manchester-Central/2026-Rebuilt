package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.subsystems.interfaces.IFeeder;

public class Feeder extends SubsystemBase implements IFeeder {
    private ChaosTalonFx m_feederMotor = new ChaosTalonFx(FeederConstants.FeederCanId, LauncherConstants.LauncherCanBus, FeederConstants.Config);

    @SuppressWarnings("unused")
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("Launcher/Feeder/Feeder Motor", m_feederMotor).withCurrentLimits();

    public Feeder() {
        m_feederMotor.applyConfig();
    }

    @Override
    public void setFeederSpeed (double speed) {
        m_feederMotor.set(speed);
    }

    @Override
    public double getFeederSpeed () {
        return m_feederMotor.get();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Feeder/Speed", getFeederSpeed());
    }
}
