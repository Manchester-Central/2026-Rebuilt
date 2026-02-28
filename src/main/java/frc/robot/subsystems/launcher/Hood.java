// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.chaos131.ctre.ChaosTalonFx;
import com.chaos131.ctre.ChaosTalonFxTuner;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.HoodConstants;
import frc.robot.subsystems.interfaces.IHood;

/** Add your docs here. */
public class Hood extends SubsystemBase implements IHood {
 private ChaosTalonFx m_hoodMotor = new ChaosTalonFx (HoodConstants.HoodCanId, LauncherConstants.LauncherCanBus, HoodConstants.HoodConfig); 

  @SuppressWarnings("unused")
    private ChaosTalonFxTuner m_hoodMotorTuner = new ChaosTalonFxTuner ("Launcher/Hood/Hood Motor", m_hoodMotor).withCurrentLimits();

  public Hood() {
    m_hoodMotor.applyConfig(); 
    if (Robot.isSimulation()) {
      var m_moi = SingleJointedArmSim.estimateMOI(HoodConstants.HoodRadius.in(Meters), HoodConstants.HoodMass.in(Kilograms));
      var m_dcMotor = DCMotor.getKrakenX44(1); // TODO: double check
      var m_dcMotorSim = new DCMotorSim(LinearSystemId.createSingleJointedArmSystem(m_dcMotor, m_moi, HoodConstants.SensorToMechanismRatio), m_dcMotor);
      m_hoodMotor.attachMotorSim(m_dcMotorSim, HoodConstants.SensorToMechanismRatio, true, ChassisReference.CounterClockwise_Positive, MotorType.KrakenX60);
    }
  }

  @Override
  public void setHoodSpeed (double speed) {
        m_hoodMotor.set(speed);
 }

  @Override
  public double getHoodSpeed () {
      return m_hoodMotor.get(); //TODO: finish IHood stuff
 }

  @Override
  public void periodic() {
      Logger.recordOutput("Hood/Speed", getHoodSpeed());
 }

  @Override
  public void setHoodAngle(Angle targetAngle) {
    m_hoodMotor.setPosition(targetAngle); //TODO: replace with actual closed loop control
  }

  @Override
  public Angle getHoodAngle() {
    return m_hoodMotor.getPosition().getValue();
  }
}
