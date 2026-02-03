// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.poses.FieldPose2026;
import com.chaos131.vision.LimelightCamera;
import com.chaos131.vision.LimelightCamera.LimelightVersion;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.defaults.ClimberDefaultCommand;
import frc.robot.commands.defaults.IntakeDefaultCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Quest;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberMech2D;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveMapleSim;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.interfaces.AbstractDrive;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.launcher.Launcher;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final AbstractDrive m_swerveDrive;
  @SuppressWarnings("unused")
  private Quest m_quest;
  @SuppressWarnings("unused")
  private Camera m_camera;
  private IClimber m_climber;
  private ClimberMech2D m_climberMech2d;
  private Launcher m_launcher;
  private Intake m_intake;
  // Controller
  private final Gamepad m_driver = new Gamepad(0);
  private final Gamepad m_operator = new Gamepad(1);

  private final boolean m_isManual = true;
  private final Trigger m_isManualTrigger = new Trigger(() -> m_isManual);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(int id, Pose2d startPose) {
    switch (Constants.currentMode) {
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

      case ARENA:
        m_swerveDrive = new DriveMapleSim(startPose);
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

    m_launcher = new Launcher();

    m_intake = new Intake ();

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
            () -> -m_driver.getLeftY(),
            () -> -m_driver.getLeftX(),
            () -> -m_driver.getRightX()));

    m_climber.setDefaultCommand(new ClimberDefaultCommand(m_climber, m_operator::getRightY, m_isManualTrigger)); 
    m_intake.setDefaultCommand(new IntakeDefaultCommand(m_intake, m_isManualTrigger, m_operator.leftTrigger()));       
    // Lock to 0° when A button is held
    m_driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_swerveDrive,
                () -> -m_driver.getLeftY(),
                () -> -m_driver.getLeftX(),
                () -> {
                    Translation2d targetPoint = FieldPose2026.HubCenter.getCurrentAlliancePose().getTranslation();
                    return targetPoint.minus(m_swerveDrive.getPose().getTranslation()).getAngle();
                }));

    // Switch to X pattern when X button is pressed
    m_driver.x().onTrue(Commands.runOnce(m_swerveDrive::stopWithX, m_swerveDrive));

    // Reset gyro to 0° when B button is pressed
    m_driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_swerveDrive.setPose(
                            new Pose2d(m_swerveDrive.getPose().getTranslation(), Rotation2d.kZero)),
                    m_swerveDrive)
                .ignoringDisable(true));

    m_driver.povUp().whileTrue(new RunCommand(() -> m_climber.setHeight(Inches.of(10)), m_climber));
    m_driver.povDown().whileTrue(new RunCommand(() -> m_climber.setHeight(Inches.of(0)), m_climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void logMech3d() {
    // Generates poses for the turret
    // - release pose at center of turret
    // - doesn't include hood, or anything else
    ArrayList<Pose3d> lst = m_launcher.generateMech3d();
    // Now generate the pose for the intake
  }
}
