package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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

    @SuppressWarnings("unused")
    private ChaosTalonFxTuner m_flywheelTuner = new ChaosTalonFxTuner("Launcher/Flywheel/Flywheel Motors", m_flywheelMotors).withAllConfigs();

    private LinearVelocity targetVelocity = MetersPerSecond.of(0);

    public Flywheel() {
        m_rightFlywheelMotor.applyConfig();
        m_leftFlywheelMotor.applyConfig();

        if (Robot.isSimulation()) {
            var m_dcMotor = DCMotor.getKrakenX60(2); // TODO: double check
            var m_moi = SingleJointedArmSim.estimateMOI(FlywheelConstants.FlyWheelDiameter.in(Meters) / 2.0, FlywheelConstants.FlywheelMass.in(Kilograms));
            var m_dcMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(m_dcMotor, m_moi, m_leftFlywheelMotor.Configuration.Feedback.SensorToMechanismRatio), m_dcMotor);
            m_leftFlywheelMotor.attachMotorSim(m_dcMotorSim, m_leftFlywheelMotor.Configuration.Feedback.SensorToMechanismRatio, true, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
        }

        m_rightFlywheelMotor.setControl(new StrictFollower(m_leftFlywheelMotor.getDeviceID()));        
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {   
        m_leftFlywheelMotor.moveAtVelocity(velocity);
    }

    public void setFlywheelVelocity(LinearVelocity linearVelocity) {
        targetVelocity = linearVelocity;
        AngularVelocity angularVelocity = RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / (Math.PI * FlywheelConstants.FlyWheelDiameter.in(Meters)));
        m_leftFlywheelMotor.moveAtVelocity(angularVelocity);
    }

    public LinearVelocity getLinearVelocity() {
        AngularVelocity angularVelocity = m_leftFlywheelMotor.getVelocity().getValue();
        return MetersPerSecond.of( angularVelocity.in(RotationsPerSecond) * (Math.PI * FlywheelConstants.FlyWheelDiameter.in(Meters)) );
    }

    public boolean atTarget() {
        if (targetVelocity == null) return false;
        return getLinearVelocity().isNear(targetVelocity, FlywheelConstants.TargetVelocityTolerance.get());
    }

   
    @Override
    public void setFlywheelSpeed(double speed) {
        m_leftFlywheelMotor.set(speed);
    }

    @Override
    public double getFlywheelSpeed() {
        // Because there's 2 motors doing the same thing, we're presuming they're going to return the same values.
        // If this proves to be incorrect, we should make a separate function in the future.
        return m_leftFlywheelMotor.get();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Launcher/FlywheelVelocity", getLinearVelocity().in(MetersPerSecond));     
        Logger.recordOutput("Launcher/FlywheelTargetVelocity", targetVelocity.in(MetersPerSecond));
    }
}
