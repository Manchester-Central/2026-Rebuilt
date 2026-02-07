package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

@Deprecated
public class TurretMapleSim extends NaivePIDMechanism implements Iturret {
    private Angle minAngle = Degrees.of(-180);
    private Angle maxAngle = Degrees.of(-180);
    private Angle targetAngle = Degrees.of(0);
    private Angle angleTolerance = Degrees.of(2);

    @Override
    public boolean setTurretSpeed(double speed) {
        return true;
    }

    public Angle getTurretSpeed() {
        return Degrees.of(getStateDifference());
    }

    public Angle getCurrentAngle() {
        return Degrees.of(getCurrentState());
    }

    public boolean atTarget() {
        return atTarget(angleTolerance);
    }

    public boolean atTarget(Angle tolerance) {
        return getCurrentAngle().isNear(targetAngle, tolerance);
    }

    @Override
    public boolean setTargetPosition(Angle angle) {
        boolean successful = true;
        if (angle.lt(minAngle)) {
            angle = minAngle;
            successful = false;
        } else if (angle.gt(maxAngle)) {
            angle = maxAngle;
            successful = false;
        }

        targetAngle = angle;
        pidController.setSetpoint(angle.in(Degrees));
        return successful;
    }
}
