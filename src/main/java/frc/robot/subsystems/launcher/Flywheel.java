package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.chaos131.util.ChaosTalonFx;
import com.chaos131.util.ChaosTalonFxTuner;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.interfaces.IFlywheel;

public class Flywheel implements IFlywheel {
    private ChaosTalonFx m_flywheelMotor = new ChaosTalonFx(LauncherConstants.FlywheelCanId, LauncherConstants.LauncherCanBus);
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("FlywheelTuner", m_flywheelMotor);

    private DashboardNumber m_kp = m_flywheelTuner.tunable("kP", ClimberConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
    private DashboardNumber m_ki = m_flywheelTuner.tunable("kI", ClimberConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
    private DashboardNumber m_kd = m_flywheelTuner.tunable("kD", ClimberConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
    private DashboardNumber m_kg = m_flywheelTuner.tunable("kG", ClimberConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
    private DashboardNumber m_ks = m_flywheelTuner.tunable("kS", ClimberConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
    private DashboardNumber m_kv = m_flywheelTuner.tunable("kV", ClimberConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
    private DashboardNumber m_ka = m_flywheelTuner.tunable("kA", ClimberConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);

    public Flywheel() {
        m_flywheelMotor.Configuration.Feedback.RotorToSensorRatio = ClimberConstants.RotorToSensorRatio;
        m_flywheelMotor.Configuration.Feedback.SensorToMechanismRatio = ClimberConstants.SensorToMechanismRatio;
        m_flywheelMotor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // TODO: dflywheelheck

        m_flywheelMotor.Configuration.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SupplyCurrentLimit.in(Amps);
        m_flywheelMotor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_flywheelMotor.Configuration.CurrentLimits.StatorCurrentLimit = ClimberConstants.StatorCurrentLimit.in(Amps);
        m_flywheelMotor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        
        m_flywheelMotor.Configuration.MotorOutput.Inverted = ClimberConstants.MotorDirection;
        m_flywheelMotor.Configuration.MotorOutput.NeutralMode = ClimberConstants.NeutralMode;

        var slot0 = new Slot0Configs();
        slot0.kP = m_kp.get();
        slot0.kI = m_ki.get();
        slot0.kD = m_kd.get();
        slot0.kG = m_kg.get();
        slot0.kS = m_ks.get();
        slot0.kV = m_kv.get();
        slot0.kA = m_ka.get();

        m_flywheelMotor.Configuration.Slot0 = slot0;

        m_flywheelMotor.applyConfig();
    }

    public void setFlywheelVelocity (AngularVelocity velocity) {
        m_flywheelMotor.moveAtVelocity(velocity);
    }

    public void setFlywheelVelocity (LinearVelocity linearVelocity) {
        AngularVelocity angularVelocity = RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / (Math.PI * FlywheelConstants.FlyWheelDiameter.in(Meters)));
        m_flywheelMotor.moveAtVelocity(angularVelocity);
    }
 
    @Override
    public void setFlywheelSpeed (double speed) {
        m_flywheelMotor.set(speed);
    }

    @Override
    public double getFlywheelSpeed () {
        return m_flywheelMotor.get();
    }
}
