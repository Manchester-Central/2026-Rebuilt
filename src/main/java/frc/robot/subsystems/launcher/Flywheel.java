package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;
import com.ctre.phoenix6.controls.StrictFollower;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FlywheelConstants;

public class Flywheel extends SubsystemBase {
    private ChaosTalonFx m_leftMainFlywheelMotor = new ChaosTalonFx(FlywheelConstants.LeftFlywheelCanId, LauncherConstants.LauncherCanBus, FlywheelConstants.LeftConfig);
    private ChaosTalonFx m_rightFollowerFlywheelMotor = new ChaosTalonFx(FlywheelConstants.RightFlywheelCanId, LauncherConstants.LauncherCanBus, FlywheelConstants.RightConfig);

    @SuppressWarnings("unused")
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("Launcher/Flywheel/Flywheel Motors", m_leftMainFlywheelMotor, m_rightFollowerFlywheelMotor).withAllConfigs();

    private LinearVelocity targetVelocity = MetersPerSecond.of(0);

    public Flywheel() {
        m_rightFollowerFlywheelMotor.applyConfig();
        m_leftMainFlywheelMotor.applyConfig();

        if (Robot.isSimulation()) {
            m_leftMainFlywheelMotor.attachMotorSim(FlywheelConstants.SimValues);
        }

        m_rightFollowerFlywheelMotor.setControl(new StrictFollower(m_leftMainFlywheelMotor.getDeviceID()));
        
        // Increase the update rate for the leader motor so the follower can respond faster (for the respective follow type. See https://www.chiefdelphi.com/t/ctre-follower-does-the-same-volts-or-the-same-control-request/513725/6)
        if (FlywheelConstants.UseTorqueCurrentFOC) {
            m_leftMainFlywheelMotor.getTorqueCurrent().setUpdateFrequency(FlywheelConstants.ClosedLoopUpdateFrequency);
        } else {
            m_leftMainFlywheelMotor.getMotorVoltage().setUpdateFrequency(FlywheelConstants.ClosedLoopUpdateFrequency);
        }
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {   
        m_leftMainFlywheelMotor.moveAtVelocity(velocity);
    }

    public void setFlywheelVelocity(LinearVelocity linearVelocity) {
        targetVelocity = linearVelocity;
        AngularVelocity angularVelocity = RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / (Math.PI * FlywheelConstants.FlyWheelDiameter.in(Meters)));
        if (FlywheelConstants.UseTorqueCurrentFOC) {
            m_leftMainFlywheelMotor.moveAtVelocityFOC(angularVelocity);
        } else {
            if (m_leftMainFlywheelMotor.getVelocity().getValue().minus(angularVelocity).in(RotationsPerSecond) > 10) {
                m_leftMainFlywheelMotor.set(1);
            } else {
                m_leftMainFlywheelMotor.moveAtVelocity(angularVelocity);
            }
        }
    }

    public LinearVelocity getLinearVelocity() {
        AngularVelocity angularVelocity = m_leftMainFlywheelMotor.getVelocity().getValue();
        return MetersPerSecond.of( angularVelocity.in(RotationsPerSecond) * (Math.PI * FlywheelConstants.FlyWheelDiameter.in(Meters)) );
    }

    public boolean atTarget() {
        if (targetVelocity == null) return false;
        return getLinearVelocity().isNear(targetVelocity, FlywheelConstants.TargetVelocityTolerance.get());
    }

    public void setFlywheelSpeed(double speed) {
        m_leftMainFlywheelMotor.set(speed);
    }

    public double getFlywheelSpeed() {
        // Because there's 2 motors doing the same thing, we're presuming they're going to return the same values.
        // If this proves to be incorrect, we should make a separate function in the future.
        return m_leftMainFlywheelMotor.get();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Launcher/FlywheelVelocity", getLinearVelocity().in(MetersPerSecond));     
        Logger.recordOutput("Launcher/FlywheelTargetVelocity", targetVelocity.in(MetersPerSecond));
    }
}
