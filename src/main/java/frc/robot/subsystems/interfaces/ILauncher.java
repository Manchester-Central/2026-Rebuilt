package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ILauncher extends Subsystem {
    public double getFlywheelSpeed();
    public void setFlywheelSpeed(double speed);
    public double getFeederSpeed();
    public void setFeederSpeed(double speed);
    public void setFlywheelVelocity(LinearVelocity velocity);
    public LinearVelocity getScoringVelocity(Pose2d currentPose);
    public boolean atTargetFlywheelVelocity();
    public LinearVelocity getPassVelocity(Pose2d currentPose);
}
