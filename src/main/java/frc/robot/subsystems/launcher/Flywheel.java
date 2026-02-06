package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;
import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FlywheelConstants;
import frc.robot.subsystems.interfaces.IFlywheel;

public class Flywheel implements IFlywheel {
    private ChaosTalonFx m_leftFlywheelMotor = new ChaosTalonFx(LauncherConstants.LeftFlywheelCanId, LauncherConstants.LauncherCanBus);
    private ChaosTalonFx m_rightFlywheelMotor = new ChaosTalonFx(LauncherConstants.RightFlywheelCanId, LauncherConstants.LauncherCanBus);
    private ChaosTalonFx[] m_flywheelMotors = {
        m_leftFlywheelMotor,
        m_rightFlywheelMotor
    };
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("FlywheelTuner", m_flywheelMotors);

    private DashboardNumber m_kp = m_flywheelTuner.tunable("kP", FlywheelConstants.kP, (config, newValue) -> config.Slot0.kP = newValue);
    private DashboardNumber m_ki = m_flywheelTuner.tunable("kI", FlywheelConstants.kI, (config, newValue) -> config.Slot0.kI = newValue);
    private DashboardNumber m_kd = m_flywheelTuner.tunable("kD", FlywheelConstants.kD, (config, newValue) -> config.Slot0.kD = newValue);
    private DashboardNumber m_kg = m_flywheelTuner.tunable("kG", FlywheelConstants.kG, (config, newValue) -> config.Slot0.kG = newValue);
    private DashboardNumber m_ks = m_flywheelTuner.tunable("kS", FlywheelConstants.kS, (config, newValue) -> config.Slot0.kS = newValue);
    private DashboardNumber m_kv = m_flywheelTuner.tunable("kV", FlywheelConstants.kV, (config, newValue) -> config.Slot0.kV = newValue);
    private DashboardNumber m_ka = m_flywheelTuner.tunable("kA", FlywheelConstants.kA, (config, newValue) -> config.Slot0.kA = newValue);

    public Flywheel() {
        prepareMotor(m_leftFlywheelMotor);
        prepareMotor(m_rightFlywheelMotor);
        // Keep these separate to control them independently.
        // There's a 50-50 chance they're different!
        m_leftFlywheelMotor.Configuration.MotorOutput.Inverted = FlywheelConstants.LeftMotorDirection;
        m_rightFlywheelMotor.Configuration.MotorOutput.Inverted = FlywheelConstants.RightMotorDirection;

        for (var motor : m_flywheelMotors)
            motor.applyConfig();
    }

    /**
     * Configures the common motor settings across all motors in the mechanism
     * @param idx index of the motor in the array
     */
    private void prepareMotor(ChaosTalonFx motor) {
        motor.Configuration.Feedback.RotorToSensorRatio = FlywheelConstants.RotorToSensorRatio;
        motor.Configuration.Feedback.SensorToMechanismRatio = FlywheelConstants.SensorToMechanismRatio;
        motor.Configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // TODO: dflywheelheck
        motor.Configuration.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.SupplyCurrentLimit.in(Amps);
        motor.Configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor.Configuration.CurrentLimits.StatorCurrentLimit = FlywheelConstants.StatorCurrentLimit.in(Amps);
        motor.Configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        motor.Configuration.MotorOutput.NeutralMode = FlywheelConstants.NeutralMode;

        var slot0 = new Slot0Configs();
        slot0.kP = m_kp.get();
        slot0.kI = m_ki.get();
        slot0.kD = m_kd.get();
        slot0.kG = m_kg.get();
        slot0.kS = m_ks.get();
        slot0.kV = m_kv.get();
        slot0.kA = m_ka.get();

        motor.Configuration.Slot0 = slot0;
    }

    public void setFlywheelVelocity (AngularVelocity velocity) {
        for (var motor : m_flywheelMotors)
            motor.moveAtVelocity(velocity);
    }

    public void setFlywheelVelocity (LinearVelocity linearVelocity) {
        AngularVelocity angularVelocity = RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / (Math.PI * FlywheelConstants.FlyWheelDiameter.in(Meters)));
        for (var motor : m_flywheelMotors)
            motor.moveAtVelocity(angularVelocity);
    }
 
    @Override
    public void setFlywheelSpeed (double speed) {
        for (var motor : m_flywheelMotors)
            motor.set(speed);
    }

    @Override
    public double getFlywheelSpeed () {
        // Because there's 2 motors doing the same thing, we're presuming they're going to return the same values.
        // If this proves to be incorrect, we should make a separate function in the future.
        return m_leftFlywheelMotor.get();
    }
}
