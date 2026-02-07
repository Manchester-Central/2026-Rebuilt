package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FlywheelConstants;
import frc.robot.subsystems.interfaces.IFlywheel;

public class Flywheel extends SubsystemBase implements IFlywheel {
    private ChaosTalonFx m_leftFlywheelMotor = new ChaosTalonFx(FlywheelConstants.LeftFlywheelCanId, LauncherConstants.LauncherCanBus, FlywheelConstants.LeftConfig);
    private ChaosTalonFx m_rightFlywheelMotor = new ChaosTalonFx(FlywheelConstants.RightFlywheelCanId, LauncherConstants.LauncherCanBus, FlywheelConstants.RightConfig);
    private ChaosTalonFx[] m_flywheelMotors = {
        m_leftFlywheelMotor,
        m_rightFlywheelMotor
    };
    
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("FlywheelTuner", m_flywheelMotors);
    private LinearVelocity targetVelocity;
    private LinearVelocity targetVelocityTolerance; 

    public Flywheel(int id) {
        m_flywheelTuner.tunableSlot0(FlywheelConstants.LeftConfig.Slot0); // Will use left config initial values, but changes will be applied to all motors
        for (var motor : m_flywheelMotors)
            motor.applyConfig();
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        for (var motor : m_flywheelMotors)
            motor.moveAtVelocity(velocity);
    }

    public void setFlywheelVelocity(LinearVelocity linearVelocity) {
        AngularVelocity angularVelocity = RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / (Math.PI * FlywheelConstants.FlyWheelDiameter.in(Meters)));
        for (var motor : m_flywheelMotors)
            motor.moveAtVelocity(angularVelocity);
    }

    public LinearVelocity getLeftLinearVelocity() {
        AngularVelocity angularVelocity = m_leftFlywheelMotor.getVelocity().getValue();
        return MetersPerSecond.of( angularVelocity.in(RotationsPerSecond) * (Math.PI * FlywheelConstants.FlyWheelDiameter.in(Meters)) );
    }

    public boolean atTargetLeft() {
        if (targetVelocity == null) return false;
        return getLeftLinearVelocity().isNear(targetVelocity, targetVelocityTolerance);
    }

    public LinearVelocity getRightLinearVelocity() {
        AngularVelocity angularVelocity = m_rightFlywheelMotor.getVelocity().getValue();
        return MetersPerSecond.of( angularVelocity.in(RotationsPerSecond) * (Math.PI * FlywheelConstants.FlyWheelDiameter.in(Meters)) );
    }

    public boolean atTargetRight() {
        if (targetVelocity == null) return false;
        return getRightLinearVelocity().isNear(targetVelocity, targetVelocityTolerance);
    }

    @Override
    public void setFlywheelSpeed(double speed) {
        for (var motor : m_flywheelMotors)
            motor.set(speed);
    }

    @Override
    public double getFlywheelSpeed() {
        // Because there's 2 motors doing the same thing, we're presuming they're going to return the same values.
        // If this proves to be incorrect, we should make a separate function in the future.
        return m_leftFlywheelMotor.get();
    }
}
