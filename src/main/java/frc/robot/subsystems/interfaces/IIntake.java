package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IIntake extends Subsystem {
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
    * Sets the angle of the pivot motor.
    */
    public void setPivotAngle(Angle angle);

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

    /**
     * NOTE: Only functional during simulation, there's no way for the robot to know how
     * many game pieces it has.
     * @return the number of game pieces in the robot (from the intake to the launcher)
     */
    public int getNumGamePieces();

    /**
     * Attempts to take a game piece from the intake/hopper
     * @return true if a piece was claimed
     */
    public boolean claimGamePiece();
}
