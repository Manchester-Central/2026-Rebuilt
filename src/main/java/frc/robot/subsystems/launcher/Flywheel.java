package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FlywheelConstants;
import frc.robot.subsystems.interfaces.IFlywheel;

public class Flywheel extends SubsystemBase implements IFlywheel {
    private ChaosTalonFx m_leftFlywheelMotor;
    private ChaosTalonFx m_rightFlywheelMotor;
    private ChaosTalonFx[] m_flywheelMotors;

    @SuppressWarnings("unused")
    private ChaosTalonFxTuner m_flywheelTuner;

    private LinearVelocity targetVelocity = MetersPerSecond.of(0);
    private LinearVelocity targetVelocityTolerance = FlywheelConstants.TargetVelocityTolerance;

    public Flywheel(int id) {
        m_leftFlywheelMotor = new ChaosTalonFx(FlywheelConstants.LeftFlywheelCanId, LauncherConstants.LauncherCanBus, FlywheelConstants.LeftConfig);
        m_rightFlywheelMotor = new ChaosTalonFx(FlywheelConstants.RightFlywheelCanId, LauncherConstants.LauncherCanBus, FlywheelConstants.RightConfig);
        m_flywheelMotors = new ChaosTalonFx[]{
            m_leftFlywheelMotor,
            m_rightFlywheelMotor
        };
        m_flywheelTuner = new ChaosTalonFxTuner("Launcher/Flywheel/Flywheel Motors", m_flywheelMotors).withAllConfigs();

        for (var motor : m_flywheelMotors) {
            motor.applyConfig();
            if (Robot.isSimulation()) {
                var m_dcMotor = DCMotor.getKrakenX60(1); // TODO: double check
                var m_moi = SingleJointedArmSim.estimateMOI(FlywheelConstants.FlyWheelDiameter.in(Meters) / 2.0, FlywheelConstants.FlywheelMass.in(Kilograms));
                var m_dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(m_dcMotor, m_moi, motor.Configuration.Feedback.SensorToMechanismRatio), m_dcMotor);
                motor.attachMotorSim(m_dcMotorSim, motor.Configuration.Feedback.SensorToMechanismRatio, true, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
            }
        }
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        for (var motor : m_flywheelMotors)
            motor.moveAtVelocity(velocity);
    }

    public void setFlywheelVelocity(LinearVelocity linearVelocity) {
        targetVelocity = linearVelocity;
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

    public void periodic() {
        Logger.recordOutput("Launcher/LeftFlywheelVelocity", getLeftLinearVelocity().in(MetersPerSecond));
        Logger.recordOutput("Launcher/RightFlywheelVelocity", getRightLinearVelocity().in(MetersPerSecond));
        Logger.recordOutput("Launcher/FlywheelTargetVelocity", targetVelocity.in(MetersPerSecond));
    }
}
