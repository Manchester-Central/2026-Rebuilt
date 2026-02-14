// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;
import com.chaos131.vision.LimelightCamera;
import com.chaos131.vision.LimelightCamera.LimelightVersion;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.climb.SetClimberHeight;
import frc.robot.commands.defaults.ClimberDefaultCommand;
import frc.robot.commands.defaults.IntakeDefaultCommand;
import frc.robot.commands.defaults.SimpleLauncherDefaultCommand;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.DeployOuttake;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.launcher.TargetHubVelocityAndLaunch;
import frc.robot.commands.launcher.TargetPassVelocityAndLaunch;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.IntakeConstants.PivotConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Quest;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberMech2D;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeMech2D;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.ISimpleLauncher;
import frc.robot.subsystems.launcher.Flywheel;
import frc.robot.subsystems.launcher.Indexer;
import frc.robot.subsystems.launcher.LauncherMech2D;
import frc.robot.subsystems.launcher.SimpleLauncher;
import frc.robot.util.PathUtil;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_swerveDrive;
  @SuppressWarnings("unused")
  private Quest m_quest;
  @SuppressWarnings("unused")
  private Camera m_camera;
  private IClimber m_climber;
  @SuppressWarnings("unused")
  private ClimberMech2D m_climberMech2d;
  @SuppressWarnings("unused")
  private IntakeMech2D m_intakeMech2d;
  @SuppressWarnings("unused")
  private LauncherMech2D m_launcherMech2D;
  private ISimpleLauncher m_launcher;
  private IIntake m_intake;

  // Controller
  private final Gamepad m_driver = new Gamepad(0);
  private final Gamepad m_operator = new Gamepad(1);

  private final DoubleSupplier m_getDriverXTranslation = () -> m_driver.getLeftY();
  private final DoubleSupplier m_getDriverYTranslation = () -> -m_driver.getLeftX();
  private final DoubleSupplier m_getDriverRotation = () -> -m_driver.getRightX();
  private final DoubleSupplier m_getDriverXTranslationSlow = () -> m_getDriverXTranslation.getAsDouble() * GeneralConstants.SlowModeMultiplier;
  private final DoubleSupplier m_getDriverYTranslationSlow = () -> m_getDriverYTranslation.getAsDouble() * GeneralConstants.SlowModeMultiplier;
  private final DoubleSupplier m_getDriverRotationSlow = () -> m_getDriverRotation.getAsDouble() *  GeneralConstants.SlowModeMultiplier;

  private boolean m_isManual = false;
  private final Trigger m_isManualTrigger = new Trigger(() -> m_isManual);
  private final Trigger m_isAutomaticTrigger = m_isManualTrigger.negate();
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (GeneralConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        m_swerveDrive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        m_quest = new Quest(m_swerveDrive);

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_swerveDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        m_swerveDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
    m_camera = new Camera("LimeLight",
            LimelightVersion.LL3G,
            LimelightCamera.LL3GSpecs(),
            () -> m_swerveDrive.getPose(),
            (data) -> m_swerveDrive.addVisionMeasurement(data.getPose2d(), data.getTimestampSeconds(), data.getDeviationMatrix()),
            () -> m_swerveDrive.getSpeed().in(MetersPerSecond),
            () -> m_swerveDrive.getRotationalSpeed().in(RotationsPerSecond));

    m_climber = new Climber();
    m_climberMech2d = new ClimberMech2D(m_climber);

    m_intake = new Intake ();
    m_intakeMech2d = new IntakeMech2D(m_intake);

    m_launcher = new SimpleLauncher(new Flywheel(), new Indexer());
    m_launcherMech2D = new LauncherMech2D(m_launcher);

    
    NamedCommands.registerCommand("DeployOuttake", new DeployOuttake(m_intake));
    NamedCommands.registerCommand("DeployIntake", new DeployIntake(m_intake));
    NamedCommands.registerCommand("RetractIntake", new RetractIntake(m_intake));
    NamedCommands.registerCommand("LaunchHub", new TargetHubVelocityAndLaunch(m_launcher, m_swerveDrive::getPose)
            .alongWith(getAimAtFieldPosesCommand(FieldPose2026.HubCenter)));
    NamedCommands.registerCommand("LaunchPass", new TargetPassVelocityAndLaunch(m_launcher, m_swerveDrive::getPose)
            .alongWith(getAimAtFieldPosesCommand(LauncherConstants.PassPoints)));
    NamedCommands.registerCommand("ClimbReach", new SetClimberHeight(m_climber, ClimberConstants.MaxExtension));
    NamedCommands.registerCommand("ClimbEngage", new SetClimberHeight(m_climber, ClimberConstants.ClimbExtension));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_swerveDrive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_swerveDrive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    m_swerveDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_swerveDrive,
            m_getDriverXTranslation,
            m_getDriverYTranslation,
            m_getDriverRotation));

    m_climber.setDefaultCommand(new ClimberDefaultCommand(m_climber, m_operator::getLeftY, m_isManualTrigger)); 
    m_intake.setDefaultCommand(new IntakeDefaultCommand(m_intake, m_isManualTrigger, m_operator.leftTrigger(), m_operator::getRightY));    
    m_launcher.setDefaultCommand(new SimpleLauncherDefaultCommand(m_launcher, m_isManualTrigger, m_operator.rightTrigger(), m_operator.rightBumper()));
    
    
    // Reset gyro to 0° when B button is pressed
    m_driver
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_swerveDrive.setPose(
                            new Pose2d(m_swerveDrive.getPose().getTranslation(), Rotation2d.kZero)),
                    m_swerveDrive)
                .ignoringDisable(true));

    m_driver
        .leftBumper()
        .and(m_isAutomaticTrigger)
        .whileTrue(
            new TargetPassVelocityAndLaunch(m_launcher, m_swerveDrive::getPose)
            .alongWith(getAimAtFieldPosesCommand(LauncherConstants.PassPoints)));
    m_driver
        .rightBumper()
        .whileTrue(getAimAtFieldPosesCommand(FieldPose2026.HubCenter));

    
    m_driver.leftTrigger().and(m_isAutomaticTrigger).whileTrue(new DeployIntake(m_intake));
    m_driver
        .rightTrigger()
        .and(m_isAutomaticTrigger)
        .whileTrue(
            new TargetHubVelocityAndLaunch(m_launcher, m_swerveDrive::getPose)
            .alongWith(getAimAtFieldPosesCommand(FieldPose2026.HubCenter)));

    m_driver.a().and(m_isAutomaticTrigger).whileTrue(new RunCommand(() -> m_intake.setPivotAngle(PivotConstants.RetractAngle), m_intake));
    m_driver.x().onTrue(Commands.runOnce(m_swerveDrive::stopWithX, m_swerveDrive));
    m_driver.y().and(m_isAutomaticTrigger).whileTrue(PathUtil.driveToPoseCommand(LauncherConstants.SafeLaunchePoint, m_swerveDrive));

    m_driver.leftStick().or(m_driver.rightStick()).toggleOnTrue(
        DriveCommands.joystickDrive(
            m_swerveDrive,
            m_getDriverXTranslationSlow,
            m_getDriverYTranslationSlow,
            m_getDriverRotationSlow)
    );

    m_operator.povUp().and(m_isAutomaticTrigger).whileTrue(new SetClimberHeight(m_climber, ClimberConstants.MaxExtension));
    m_operator.povRight().and(m_isAutomaticTrigger).whileTrue(new SetClimberHeight(m_climber, ClimberConstants.ClimbExtension));
    m_operator.povDown().and(m_isAutomaticTrigger).whileTrue(new SetClimberHeight(m_climber, ClimberConstants.MinExtension));

    m_operator.y().whileTrue(new InstantCommand(() -> m_quest.resetPose(m_camera.getBotPose3d())));

    m_operator.start().onTrue(new InstantCommand((() -> m_isManual = true)));
    m_operator.back().onTrue(new InstantCommand((() -> m_isManual = false)));
  }

  private Command getAimAtFieldPosesCommand(FieldPose2026... poses) {
    return DriveCommands.joystickDriveAtAngle(
        m_swerveDrive,
       DriverStation.isAutonomous() ? () -> 0 :  m_getDriverXTranslation,
       DriverStation.isAutonomous() ? () -> 0 : m_getDriverYTranslation,
        () -> {
            FieldPose targetPose = FieldPose.getClosestPose(m_swerveDrive.getPose(), poses);
            return targetPose.getTargetAngleForRobot(m_swerveDrive.getPose());
        }
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Camera getCamera() {
    return m_camera;
  }

  public Drive getSwerveDrive() {
    return m_swerveDrive;
  }

  public Quest getQuest() {
    return m_quest;
  }
}
