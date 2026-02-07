package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.interfaces.IIntake;

/**
 * Leverages existing Flywheel behavior, but attaches 
 */
public class MapleSimFlywheel extends Flywheel {
    protected double ballLaunchInterval = 0.3;
    protected IIntake intake;

    public MapleSimFlywheel(int id, IIntake intake) {
        super(id);
        this.intake = intake;
    }

    @Override
    public void periodic() {
        if (this.m_leftFlywheelMotor) {
            //
        }
    }
}
