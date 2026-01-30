package frc.robot.subsystems.launcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NaivePIDMechanism extends SubsystemBase {
    // We should be using actual motor APIs here, but because there's too
    // many motors we would use all the canbus IDs, we're doing this with
    // software PID and pretending there's a motor in here somewhere.
    protected PIDController pidController = new PIDController(0.3, 0.0, 0.0);
    /* Stores the [currentAngle, previousAngle] */
    protected double[] positionHistory = new double[]{0, 0};
    protected final int PREVIOUS = 1;
    protected final int CURRENT = 0;

    protected double getStateDifference() {
        return positionHistory[CURRENT]-positionHistory[PREVIOUS];
    }

    protected double getCurrentState() {
        return positionHistory[CURRENT];
    }

    protected void setTargetState(double target) {
        pidController.setSetpoint(target);
    }

    @Override
    public void periodic() {
        var newstate = pidController.calculate(getCurrentState());
        positionHistory[PREVIOUS] = positionHistory[CURRENT];
        positionHistory[CURRENT] = newstate;
    }
}
