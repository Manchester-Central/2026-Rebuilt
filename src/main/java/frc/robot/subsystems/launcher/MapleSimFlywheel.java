package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotDimensions;
import frc.robot.subsystems.MultiplayerSim.MultiplayerArena;
import frc.robot.subsystems.interfaces.AbstractDrive;
import frc.robot.subsystems.interfaces.IIntake;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/**
 * Leverages existing Flywheel behavior, but attaches 
 */
public class MapleSimFlywheel extends Flywheel {
    protected double ballLaunchInterval = 0.3;

    // Left Data
    protected Timer leftBallTimer = new Timer();
    protected Transform3d leftOffset = new Transform3d(
        Meters.of(0.4),
        Meters.of(0.1),
        Meters.of(0.4),
        Rotation3d.kZero);

    // Right Data
    protected Timer rightBallTimer = new Timer();
    protected Transform3d rightOffset = new Transform3d(
        Meters.of(0.4),
        Meters.of(-0.1),
        Meters.of(0.4),
        Rotation3d.kZero);

    protected AbstractDrive drive;
    protected IIntake intake;

    public MapleSimFlywheel(int id, AbstractDrive drive, IIntake intake) {
        super(id);
        this.intake = intake;
        this.drive = drive;
    }

    @Override
    public void periodic() {
        if (atTargetLeft() && leftBallTimer.hasElapsed(ballLaunchInterval)) {
            if (intake.claimGamePiece()) {
                leftBallTimer.reset();
                launchGamePiece(drive.getPose(), leftOffset, MetersPerSecond.of(10));
            }
        }

        if (atTargetRight() && rightBallTimer.hasElapsed(ballLaunchInterval)) {
            if (intake.claimGamePiece()) {
                rightBallTimer.reset();
                launchGamePiece(drive.getPose(), rightOffset, MetersPerSecond.of(10));
            }
        }
    }

    private void launchGamePiece(Pose2d robotPose, Transform3d launcherOffset, LinearVelocity launcherSpeed) {
        Pose3d launcherPose = new Pose3d(robotPose);
        launcherPose = launcherPose.transformBy(launcherOffset);

        MultiplayerArena.Instance.addGamePieceProjectile(new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            new Translation2d(launcherOffset.getX(), launcherOffset.getY()),
            drive.getChassisSpeeds(),
            Rotation2d.kZero,
            Meters.of(launcherOffset.getZ()),
            launcherSpeed,
            RobotDimensions.FixedHoodAngle));
    }
}
