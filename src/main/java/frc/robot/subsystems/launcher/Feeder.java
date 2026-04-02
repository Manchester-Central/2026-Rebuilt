package frc.robot.subsystems.launcher;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.subsystems.interfaces.AbstractFeeder;

public class Feeder extends AbstractFeeder {
    private ChaosTalonFx m_topFeederMotor = new ChaosTalonFx(FeederConstants.TopFeederCanId, LauncherConstants.LauncherCanBus, FeederConstants.TopConfig);
    private ChaosTalonFx m_bottomFeederMotor = new ChaosTalonFx(FeederConstants.BottomFeederCanId, LauncherConstants.LauncherCanBus, FeederConstants.BottomConfig);

    private DigitalInput m_beamSensor = new DigitalInput(FeederConstants.SensorIndex); // TODO: check input channel

    @SuppressWarnings("unused")
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("Launcher/Feeder/Feeder Motors", m_topFeederMotor, m_bottomFeederMotor).withCurrentLimits();

    public Feeder(int id) {
        super(id);
        m_topFeederMotor.applyConfig();
        m_bottomFeederMotor.applyConfig();
    }

    @Override
    public void setFeederSpeed (double speed) {
        m_topFeederMotor.set(speed);
        m_bottomFeederMotor.set(speed);
    }

    @Override
    public void setFeederSpeed (double bottomSpeed, double topSpeed) {
        m_topFeederMotor.set(topSpeed);
        m_bottomFeederMotor.set(bottomSpeed);
    }

    @Override
    public double getFeederSpeed () {
        return m_topFeederMotor.get();
    }

    @Override
    public boolean doesFeederHaveFuel() {
        return !m_beamSensor.get(); // TODO: test if not is needed
    }
}
