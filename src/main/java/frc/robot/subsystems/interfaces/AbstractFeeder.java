package frc.robot.subsystems.interfaces;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractFeeder extends SubsystemBase {
  public abstract void setFeederSpeed(double speed);
  public abstract void setFeederSpeed(double bottomSpeed, double topSpeed);
  public abstract double getFeederSpeed();
  public abstract boolean doesFeederHaveFuel();
  
  @Override
  public void periodic() {
    Logger.recordOutput("Feeder/Speed", getFeederSpeed());
  }
}
