package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.Angle;

public interface Iturret {
    public boolean setTargetPosition (Angle targetAngle);
    public boolean setTurretSpeed (double speed);
}
