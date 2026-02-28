package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.subsystems.interfaces.IFeeder;

public class Feeder extends SubsystemBase implements IFeeder {
    private ChaosTalonFx m_topFeederMotor = new ChaosTalonFx(FeederConstants.TopFeederCanId, LauncherConstants.LauncherCanBus, FeederConstants.TopConfig);
    private ChaosTalonFx m_bottomFeederMotor = new ChaosTalonFx(FeederConstants.BottomFeederCanId, LauncherConstants.LauncherCanBus, FeederConstants.BottomConfig);

    @SuppressWarnings("unused")
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("Launcher/Feeder/Feeder Motors", m_topFeederMotor, m_bottomFeederMotor).withCurrentLimits();

    public Feeder(int id) {
        m_topFeederMotor.applyConfig();
        m_bottomFeederMotor.applyConfig();
    }

    @Override
    public void setFeederSpeed (double speed) {
        m_topFeederMotor.set(speed);
        m_bottomFeederMotor.set(speed);
    }

    @Override
    public double getFeederSpeed () {
        return m_topFeederMotor.get();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Feeder/Speed", getFeederSpeed());
    }
}
