package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ISimpleLauncher extends Subsystem {
    public double getFlywheelSpeed();
    public void setFlywheelSpeed(double speed);
    public double getIndexerSpeed();
    public void setIndexerSpeed(double speed);
}
