package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Deprecated
public class NaivePIDMechanism extends SubsystemBase {
    // We should be using actual motor APIs here, but because there's too
    // many motors we would use all the canbus IDs, we're doing this with
    // software PID and pretending there's a motor in here somewhere.
    protected PIDController pidController = new PIDController(0.3, 0.0, 0.0);
    protected double previousPosition;
    protected double currentPosition;

    protected double getStateDifference() {
        return currentPosition-previousPosition;
    }

    protected double getCurrentState() {
        return currentPosition;
    }

    protected void setTargetState(double target) {
        pidController.setSetpoint(target);
    }

    @Override
    public void periodic() {
        var newstate = pidController.calculate(getCurrentState());
        previousPosition = currentPosition;
        currentPosition = newstate;
    }
}
