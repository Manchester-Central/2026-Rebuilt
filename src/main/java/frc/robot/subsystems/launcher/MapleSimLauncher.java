package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotDimensions;
import frc.robot.subsystems.MultiplayerSim.MultiplayerArena2026;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.interfaces.AbstractDrive;
import frc.robot.subsystems.interfaces.AbstractFeeder;

/**
 * Leverages existing Flywheel behavior, but attaches 
 */
public class MapleSimLauncher extends Launcher {
    protected double ballLaunchInterval = 0.3;

    // Left Data
    protected Timer leftBallTimer = new Timer();
    protected Transform3d leftOffset = new Transform3d(
        Meters.of(0.4), // forward
        Meters.of(0.1), // left
        Meters.of(0.4), // up
        Rotation3d.kZero);

    // Right Data
    protected Timer rightBallTimer = new Timer();
    protected Transform3d rightOffset = new Transform3d(
        Meters.of(0.4),
        Meters.of(-0.1),
        Meters.of(0.4),
        Rotation3d.kZero);

    protected AbstractDrive drive;
    protected Intake intake;

    public MapleSimLauncher(Flywheel flywheel, AbstractFeeder feeder, Hood hood, AbstractDrive drive, Intake intake) {
        super(flywheel, feeder, hood, drive);
        this.intake = intake;
        this.drive = drive;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (atTargetFlywheelVelocity() && leftBallTimer.hasElapsed(ballLaunchInterval)) {
            if (intake.claimGamePiece()) {
                leftBallTimer.reset();
                launchGamePiece(drive.getPose(), leftOffset, MetersPerSecond.of(10));
            }
        }
    }

    private void launchGamePiece(Pose2d robotPose, Transform3d launcherOffset, LinearVelocity launcherSpeed) {
        Pose3d launcherPose = new Pose3d(robotPose);
        launcherPose = launcherPose.transformBy(launcherOffset);

        MultiplayerArena2026.Instance.addGamePieceProjectile(new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            new Translation2d(launcherOffset.getX(), launcherOffset.getY()),
            drive.getChassisSpeeds(),
            robotPose.getRotation(),
            Meters.of(launcherOffset.getZ()),
            launcherSpeed,
            RobotDimensions.FixedHoodAngle));
    }
}
