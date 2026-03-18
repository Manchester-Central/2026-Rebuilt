package frc.robot.subsystems.launcher;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interfaces.AbstractFeeder;

public class FakeFeeder extends AbstractFeeder {
  protected Intake intake;

  protected double topspeed = 0;
  protected double bottomspeed = 0;

  public FakeFeeder(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void setFeederSpeed(double speed) {
    topspeed = bottomspeed = speed;
  }

  @Override
  public void setFeederSpeed(double bottomSpeed, double topSpeed) {
    topspeed = topSpeed;
    bottomspeed = bottomSpeed;
  }

  @Override
  public double getFeederSpeed() {
    return topspeed;
  }

  @Override
  public boolean doesFeederHaveFuel() {
    return intake.getNumGamePieces() > 0;
  }
}
