package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ILauncher extends Subsystem {
    public double getFlywheelSpeed();
    public void setFlywheelSpeed(double speed);
    public double getFeederSpeed();
    public void setFeederSpeed(double speed);
    public void setFlywheelVelocity(LinearVelocity velocity);
    public LinearVelocity getScoringVelocitySetAngle(Pose2d currentPose);
    public boolean atTargetFlywheelVelocity();
    public LinearVelocity getPassVelocitySetAngle(Pose2d currentPose);
    public double getHoodSpeed();
    public void setHoodSpeed(double speed);
    public void setHoodAngle(Angle targetAngle);
    public Angle getHoodAngle();
    public boolean atTargetHoodAngle();
    public boolean atTargets();
    public void setTargets(LinearVelocity velocity, Angle angle);
    public boolean doesFeederHaveFuel();
    public LinearVelocity getVelocityForTargetSetHeight(IDrive swerveDrive, Pose2d targetPose, Distance targetHeight);
    public Angle getPitchForTarget(IDrive swerveDrive, Pose2d targetPose, Distance targetHeight);
    public Angle getYawForTarget(IDrive swerveDrive, Pose2d targetPose, Distance targetHeight);
}
