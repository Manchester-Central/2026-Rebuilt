package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FlywheelConstants;
import frc.robot.subsystems.interfaces.IFlywheel;

public class Flywheel implements IFlywheel {
    private ChaosTalonFx m_leftFlywheelMotor = new ChaosTalonFx(FlywheelConstants.LeftFlywheelCanId, LauncherConstants.LauncherCanBus, FlywheelConstants.LeftConfig);
    private ChaosTalonFx m_rightFlywheelMotor = new ChaosTalonFx(FlywheelConstants.RightFlywheelCanId, LauncherConstants.LauncherCanBus, FlywheelConstants.RightConfig);
    private ChaosTalonFx[] m_flywheelMotors = {
        m_leftFlywheelMotor,
        m_rightFlywheelMotor
    };
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("FlywheelTuner", m_flywheelMotors);

    public Flywheel() {
        m_flywheelTuner.tunableSlot0(FlywheelConstants.LeftConfig.Slot0); // Will use left config initial values, but changes will be applied to all motors
        for (var motor : m_flywheelMotors)
            motor.applyConfig();
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
