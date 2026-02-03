package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.chaos131.util.ChaosTalonFx;
import com.chaos131.util.ChaosTalonFxTuner;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.interfaces.IFlywheel;

public class Flywheel implements IFlywheel {
    private ChaosTalonFx[] m_flywheelMotor = {
        new ChaosTalonFx(LauncherConstants.FlywheelCanId, LauncherConstants.LauncherCanBus),
        new ChaosTalonFx(LauncherConstants.FlywheelCanId2, LauncherConstants.LauncherCanBus)
    };
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("FlywheelTuner", m_flywheelMotor);

    private DashboardNumber m_kp = m_flywheelTuner.tunable("kP", FlywheelConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
    private DashboardNumber m_ki = m_flywheelTuner.tunable("kI", FlywheelConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
    private DashboardNumber m_kd = m_flywheelTuner.tunable("kD", FlywheelConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
    private DashboardNumber m_kg = m_flywheelTuner.tunable("kG", FlywheelConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
    private DashboardNumber m_ks = m_flywheelTuner.tunable("kS", FlywheelConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
    private DashboardNumber m_kv = m_flywheelTuner.tunable("kV", FlywheelConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
    private DashboardNumber m_ka = m_flywheelTuner.tunable("kA", FlywheelConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);

    public Flywheel() {
        configureMotor(0);
        configureMotor(1);
        m_flywheelMotor[0].Configuration.MotorOutput.Inverted = FlywheelConstants.MotorDirection[0];
        m_flywheelMotor[1].Configuration.MotorOutput.Inverted = FlywheelConstants.MotorDirection[1];

        for (var motor : m_flywheelMotor)
            motor.applyConfig();
    }

    /**
     * Configures the common motor settings across all motors in the mechanism
     * @param idx index of the motor in the array
     */
    private void configureMotor(int idx) {
        m_flywheelMotor[idx].Configuration.Feedback.RotorToSensorRatio = FlywheelConstants.RotorToSensorRatio;
        m_flywheelMotor[idx].Configuration.Feedback.SensorToMechanismRatio = FlywheelConstants.SensorToMechanismRatio;
        m_flywheelMotor[idx].Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // TODO: dflywheelheck

        m_flywheelMotor[idx].Configuration.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.SupplyCurrentLimit.in(Amps);
        m_flywheelMotor[idx].Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_flywheelMotor[idx].Configuration.CurrentLimits.StatorCurrentLimit = FlywheelConstants.StatorCurrentLimit.in(Amps);
        m_flywheelMotor[idx].Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        
        m_flywheelMotor[idx].Configuration.MotorOutput.NeutralMode = FlywheelConstants.NeutralMode;

        var slot0 = new Slot0Configs();
        slot0.kP = m_kp.get();
        slot0.kI = m_ki.get();
        slot0.kD = m_kd.get();
        slot0.kG = m_kg.get();
        slot0.kS = m_ks.get();
        slot0.kV = m_kv.get();
        slot0.kA = m_ka.get();
        slot0.GravityType = GravityTypeValue.Elevator_Static;

        m_flywheelMotor[idx].Configuration.Slot0 = slot0;
    }

    public void setFlywheelVelocity (AngularVelocity velocity) {
        for (var motor : m_flywheelMotor)
            motor.moveAtVelocity(velocity);
    }

    public void setFlywheelVelocity (LinearVelocity linearVelocity) {
        AngularVelocity angularVelocity = RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / (Math.PI * FlywheelConstants.FlyWheelDiameter.in(Meters)));
        for (var motor : m_flywheelMotor)
            motor.moveAtVelocity(angularVelocity);
    }
 
    @Override
    public void setFlywheelSpeed (double speed) {
        for (var motor : m_flywheelMotor)
            motor.set(speed);
    }

    @Override
    public double getFlywheelSpeed () {
        // Because there's 2 motors doing the same thing, we're presuming they're going to return the same values.
        // If this proves to be incorrect, we should make a separate function down the line.
        return m_flywheelMotor[0].get();
    }
}
