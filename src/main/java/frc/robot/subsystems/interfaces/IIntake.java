package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Angle;

public interface IIntake {
    /**
     * Sets the speed of the intake mechanism.
     * Applies the same speed to both the main and kicker roller (but not the pivot roller)
     * 
     * @param speed double in the range of [-1.0, 1.0]
     */
    public void setRollerSpeed(double speed);

    /**
     * Sets the speed of the intake mechanism.
     * Applies the same speed to both the main and kicker roller (but not the pivot roller)
     * 
     * @param speed double in the range of [-1.0, 1.0]
     */
    public void setPivotSpeed(double speed);

    /**
     * Sets the speed of the intake mechanism. Controls the roller and kicker independently.
     * 
     * @param roller double in the range of [-1.0, 1.0]
     * @param kicker double in the range of [-1.0, 1.0]
     */
    // public void setIntakeSpeed(double roller, double kicker);

    /**
     * @return the current speed of the main intake roller, in the range of [-1.0, 1.0]
     */
    public double getRollerSpeed();

    /**
     * @return the current speed of the intake's kicker roller, in the range of [-1.0, 1.0]
     */
    // public double getIntakeKickerSpeed();

    /**
     * @return the current angle of the main intake pivot
     */
    public Angle getPivotAngle();

    /**
     * Deploys the Intake, if it isn't already.
     */
    public void deploy();

    /**
     * Retracts the Intake, if it isn't already.
     */
    public void retract();
}
