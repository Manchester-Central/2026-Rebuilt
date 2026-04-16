// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.chaos131.gamepads.Gamepad;
import com.chaos131.poses.FieldPose;
import com.chaos131.poses.FieldPose2026;
import com.chaos131.util.DriveDirection;
import com.chaos131.vision.LimelightCamera;
import com.chaos131.vision.LimelightCamera.LimelightVersion;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RepeatedConditionalCommand;
import frc.robot.commands.defaults.IntakeDefaultCommand;
import frc.robot.commands.defaults.LauncherDefaultCommand;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.DeployOuttake;
import frc.robot.commands.intake.RetractBump;
import frc.robot.commands.intake.RetractIntake;
import frc.robot.commands.launcher.AimHubAndLaunchJostle;
import frc.robot.commands.launcher.AimHubAndLaunchTable;
import frc.robot.commands.launcher.AimHubAndLaunchTunable;
import frc.robot.commands.launcher.AimPassAndLaunchJostle;
import frc.robot.commands.launcher.AimPassAndLaunchSetAngle;
import frc.robot.commands.manual.IntakeManualCommand;
import frc.robot.commands.manual.LauncherManualCommand;
import frc.robot.constants.FieldDimensions;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.FeederConstants;
import frc.robot.constants.LauncherConstants.HoodConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Quest;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeMech2D;
import frc.robot.subsystems.launcher.Feeder;
import frc.robot.subsystems.launcher.Flywheel;
import frc.robot.subsystems.launcher.Hood;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherMech2D;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_swerveDrive;
  private Quest m_quest;
  private Camera m_camera;
  // private IClimber m_climber;
  // @SuppressWarnings("unused")
  // // private ClimberMech2D m_climberMech2d;
  // @SuppressWarnings("unused")
  private IntakeMech2D m_intakeMech2d;
  @SuppressWarnings("unused")
  private LauncherMech2D m_launcherMech2D;
  private Launcher m_launcher;
  private Intake m_intake;

  // Controller
  private final Gamepad m_driver = new Gamepad(0);
  private final Gamepad m_operator = new Gamepad(1);

  private final DoubleSupplier m_getDriverXTranslation = () -> m_driver.getLeftY();
  private final DoubleSupplier m_getDriverYTranslation = () -> -m_driver.getLeftX();
  private final DoubleSupplier m_getDriverRotation = () -> -m_driver.getRightX();
  private final DoubleSupplier m_getDriverXTranslationSlow = () -> m_getDriverXTranslation.getAsDouble() * GeneralConstants.SlowModeMultiplier;
  private final DoubleSupplier m_getDriverYTranslationSlow = () -> m_getDriverYTranslation.getAsDouble() * GeneralConstants.SlowModeMultiplier;
  private final DoubleSupplier m_getDriverXTranslationMoveLaunch = () -> m_getDriverXTranslation.getAsDouble() * GeneralConstants.MoveLaunchDriveMultiplier;
  private final DoubleSupplier m_getDriverYTranslationMoveLaunch = () -> m_getDriverYTranslation.getAsDouble() * GeneralConstants.MoveLaunchDriveMultiplier;
  private final DoubleSupplier m_getDriverRotationSlow = () -> m_getDriverRotation.getAsDouble() *  GeneralConstants.SlowModeMultiplier;

  private boolean m_isManual = false;
  private final Trigger m_isManualTrigger = new Trigger(() -> m_isManual);
  private final Trigger m_isAutomaticTrigger = m_isManualTrigger.negate();
  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

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
    m_camera = new Camera("limelight",
            LimelightVersion.LL3G,
            LimelightCamera.LL3GSpecs(),
            () -> m_swerveDrive.getPose(),
            (data) -> m_swerveDrive.addVisionMeasurement(data.getPose2d(), data.getTimestampSeconds(), data.getDeviationMatrix()),
            () -> m_swerveDrive.getSpeed().in(MetersPerSecond),
            () -> m_swerveDrive.getRotationalSpeed().in(RotationsPerSecond));

    // m_climber = new Climber();
    // m_climberMech2d = new ClimberMech2D(m_climber);

    m_intake = new Intake ();
    m_intakeMech2d = new IntakeMech2D(m_intake);

    m_launcher = new Launcher(new Flywheel(), new Feeder(), new Hood(), m_swerveDrive);
    m_launcherMech2D = new LauncherMech2D(m_launcher);

    addAutos();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("DeployOuttake", new DeployOuttake(m_intake));
    NamedCommands.registerCommand("DeployIntake", new DeployIntake(m_intake));
    NamedCommands.registerCommand("RetractIntake", new RetractIntake(m_intake));
    NamedCommands.registerCommand("RetractBump", new RetractBump(m_intake));
    NamedCommands.registerCommand("LaunchHub", new AimHubAndLaunchJostle(m_launcher, m_swerveDrive, m_intake)
            .deadlineFor(getAimAtFieldPosesMovingCommand(FieldPose2026.HubCenter)));
    NamedCommands.registerCommand("LaunchPass", new AimPassAndLaunchSetAngle(m_launcher, m_swerveDrive, m_intake)
            .deadlineFor(getAimAtFieldPosesCommand(LauncherConstants.PassPoints)));
    // NamedCommands.registerCommand("ClimbReach", new SetClimberHeight(m_climber, ClimberConstants.MaxExtension));
    // NamedCommands.registerCommand("ClimbEngage", new SetClimberHeight(m_climber, ClimberConstants.ClimbExtension));
  }

  private void addAutos() {
    configureNamedCommands();

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
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDefaultCommands();
    configureDriverButtons();
    configureOperatorButtons();
  }

  private void configureDefaultCommands() {
    // Default command, normal field-relative drive
    m_swerveDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_swerveDrive,
            m_getDriverXTranslation,
            m_getDriverYTranslation,
            m_getDriverRotation));

    // m_climber.setDefaultCommand(switchAutomaticOrManual(
    //   new ClimberDefaultCommand(m_climber), // automatic
    //   new ClimberManualCommand(m_climber)   // manual
    // )); 

    m_intake.setDefaultCommand(switchAutomaticOrManual(
      new IntakeDefaultCommand(m_intake),   // automatic
      new IntakeManualCommand(m_intake)     // manual
    ));

    m_launcher.setDefaultCommand(switchAutomaticOrManual(
      new LauncherDefaultCommand(m_launcher), // automatic
      new LauncherManualCommand(m_launcher)   // manual
    ));
  }

  private void configureDriverButtons() {

    // Reset gyro to directions based on pov buttons
    m_driver.povUp().onTrue(resetPoseCommand(DriveDirection.Away));
    m_driver.povLeft().onTrue(resetPoseCommand(DriveDirection.Left));
    m_driver.povRight().onTrue(resetPoseCommand(DriveDirection.Right));
    m_driver.povDown().onTrue(resetPoseCommand(DriveDirection.Towards));

    // LB: Aim and pass (if manual mode, only aim drive)
    m_driver.leftBumper().whileTrue(switchAutomaticOrManual(
      // Automatic
      new RunCommand(() -> m_intake.setRollerSpeed(IntakeConstants.OuttakeRollerSpeed.get()), m_intake),
      // manual
      new InstantCommand() // getAimAtAngleCommand(() -> DriveDirection.Towards.getAllianceAngle().getMeasure())
    ));
    // LT: Intake
    m_driver.leftTrigger().and(m_isAutomaticTrigger).whileTrue(switchAutomaticOrManual(
      // Automatic
      new DeployIntake(m_intake).alongWith(new RunCommand(() -> m_launcher.setFeederSpeed(0, FeederConstants.UnjamSpeed.get()), m_launcher)),
      // Manual
      new InstantCommand()
    ));
    // RB: Aim at hub
    m_driver.rightBumper().whileTrue(getAimWithXCommand(FieldPose2026.HubCenter));
    // RT: Aim and score in hub (if manual mode, only aim drive)
    m_driver.rightTrigger().whileTrue(switchAutomaticOrManual(
      // automatic
      new AimHubAndLaunchJostle(m_launcher, m_swerveDrive, m_intake)
        .alongWith(getAimWithXCommand(FieldPose2026.HubCenter)),
      // manual
      getAimAtFieldPosesCommand(FieldPose2026.HubCenter)
    ));

    // A: retract intake
    m_driver.a().whileTrue(switchAutomaticOrManual(
      // automatic
      new RetractIntake(m_intake),
      // manual
      new InstantCommand()
    ));
    // B: 
    m_driver.b().whileTrue(switchAutomaticOrManual(
    new AimHubAndLaunchTable(m_launcher, m_swerveDrive, m_intake)
    .alongWith(getAimAtFieldPosesMovingCommand(FieldPose2026.HubCenter)),
     getAimAtFieldPosesCommand(FieldPose2026.HubCenter))); 
    // X: Set drive to X mode (defensive position)
    m_driver.x().onTrue(Commands.runOnce(m_swerveDrive::stopWithX, m_swerveDrive));
    // Y: Drive to the safe launch point
    m_driver.y().whileTrue(getAimForwardCommand());

    // Left or Right stick press: toggles slow mode
    m_driver.leftStick().or(m_driver.rightStick()).toggleOnTrue(
        DriveCommands.joystickDrive(
            m_swerveDrive,
            m_getDriverXTranslationSlow,
            m_getDriverYTranslationSlow,
            m_getDriverRotationSlow)
    );

    // Start: 
    m_driver.start();
    // Back: 
    m_driver.back();
  }

  private void configureOperatorButtons() {

    // RB: manually prep flywheels (no feeder)
    m_operator.rightBumper().whileTrue(switchAutomaticOrManual(
      // automatic
      new InstantCommand(),
      // manual
      new RunCommand(() -> m_launcher.setFlywheelSpeed(LauncherConstants.LauncherSpeed.get()), m_launcher) 
    ));;
    // RT: manually run flywheels and feeder
    m_operator.rightTrigger().whileTrue(switchAutomaticOrManual(
      // automatic
      new AimHubAndLaunchTunable(m_launcher, m_swerveDrive, m_intake),
      // manual
      // new RunCommand(() -> {
      //   m_launcher.setFeederSpeed(FeederConstants.FeederSpeed.get());
      //   m_launcher.setFlywheelSpeed(LauncherConstants.LauncherSpeed.get());
      // }, m_launcher) 
      new AimHubAndLaunchTunable(m_launcher, m_swerveDrive, m_intake)
    ));
    // LB: Unjam intake (automatic and manual mode)
    m_operator.leftBumper().whileTrue(switchAutomaticOrManual(
      // Automatic
      new AimPassAndLaunchJostle(m_launcher, m_swerveDrive, m_intake)
        .alongWith(getAimWithXCommand(() -> DriveDirection.Towards.getAllianceAngle().getMeasure())),
      // manual
      new RunCommand(() -> m_intake.setRollerSpeed(IntakeConstants.OuttakeRollerSpeed.get()), m_intake)
    ));
    // LT: controls manually running the intake rollers
    m_operator.leftTrigger().whileTrue(switchAutomaticOrManual(
      // automatic
      new InstantCommand(),
      // manual
      new RunCommand(() -> m_intake.setRollerSpeed(IntakeConstants.IntakeRollerSpeed.get()), m_intake)
    ));

    // Right Y: controls manual intake pivot
    m_operator.rightY().whileTrue(switchAutomaticOrManual(
      // automatic
      new InstantCommand(),
      // manual
      new RunCommand(() -> m_intake.setPivotSpeed(m_operator.getRightY() * -1.0 * IntakeConstants.ManualPivotSpeedMultiplier.get()), m_intake) // TODO: does -1 make sense for this one?
    ));

    // Left Y: controls manual climb
    m_operator.leftY().whileTrue(switchAutomaticOrManual(
      // automatic
      new InstantCommand(),
      // manual
      new RunCommand(() -> m_launcher.setHoodSpeed(m_operator.getLeftY() * -1.0 * HoodConstants.ManualHoodSpeedMultiplier.get()), m_launcher) // TODO: -1 seems wrong. Implies motor is inverted correctly
    ));

    // // POV up: controls climber to up position (with manaual fixed move too)
     m_operator.povUp().whileTrue (new InstantCommand(
     ()-> m_launcher.increaseFlywheelMultiplier()
     ));
   
    // // POV left:
    // m_operator.povLeft();
    // // POV right: moves to the climb position
    // m_operator.povRight().whileTrue(switchAutomaticOrManual(
    //   // automatic
    //   new SetClimberHeight(m_climber, ClimberConstants.ClimbExtension),
    //   // manual
    //   new InstantCommand()
    // ));
    // // POV down: controls climber to down position (with manaual fixed move too)
     m_operator.povDown().whileTrue(new InstantCommand(
      ()-> m_launcher.decreaseFlywheelMultiplier()
     ));

    // A: Move hood up
    m_operator.a().whileTrue(switchAutomaticOrManual(
      // automatic
      new RunCommand(()-> m_launcher.setHoodAngle(HoodConstants.HoodMinAngle), m_launcher), 
      // manual
      new RunCommand(() -> m_launcher.setHoodSpeed(-HoodConstants.HoodSpeed.get()), m_launcher)
    ));
    // B: Move hood down
    m_operator.b().whileTrue(switchAutomaticOrManual(
      // automaticm
      new RunCommand(()-> m_launcher.setHoodAngle(HoodConstants.HoodMaxAngle), m_launcher), 
      // manual
      new RunCommand(() -> m_launcher.setHoodSpeed(HoodConstants.HoodSpeed.get()), m_launcher)
    ));
    // X: Unjam launcher (automatic and manual mode)
    m_operator.x().whileTrue(new RunCommand(() -> {
      m_launcher.setFeederSpeed(FeederConstants.UnjamSpeed.get());
      m_launcher.setFlywheelSpeed(LauncherConstants.UnjamSpeed.get());
    }, m_launcher));
    // Y: resets quest and odometry pose to limelight pose
    m_operator.y().whileTrue(new RunCommand(() -> {
      var botpose = m_camera.getBotPose3d();
      if (botpose != null && !botpose.equals(new Pose3d())){
        m_quest.resetPose(botpose);
        m_swerveDrive.setPose(botpose.toPose2d());
      }
    }).ignoringDisable(true));


    // Start: enables manual mode
    m_operator.start().onTrue(new InstantCommand((() -> m_isManual = true)));
    // Back: enabled auto mode
    m_operator.back().onTrue(new InstantCommand((() -> m_isManual = false)));

    m_operator.leftStick().onTrue(new InstantCommand(() -> m_quest.setUseForOdometry(false)));
  }

  private Command switchAutomaticOrManual(Command autoCommand, Command manualCommand) {
    return new ConditionalCommand(autoCommand, manualCommand, m_isAutomaticTrigger);
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

  private Command getAimAtAngleCommand(Supplier<Angle> angle) {
    return DriveCommands.joystickDriveAtAngle(
        m_swerveDrive,
        DriverStation.isAutonomous() ? () -> 0 :  m_getDriverXTranslation,
        DriverStation.isAutonomous() ? () -> 0 : m_getDriverYTranslation,
        () -> Rotation2d.fromDegrees(angle.get().in(Degrees))
    );
  }

  private Command getAimAtFieldPosesMovingCommand(FieldPose2026... poses) {
    return DriveCommands.joystickDriveAtAngle(
        m_swerveDrive,
        DriverStation.isAutonomous() ? () -> 0 :  m_getDriverXTranslationMoveLaunch,
        DriverStation.isAutonomous() ? () -> 0 : m_getDriverYTranslationMoveLaunch,
        () -> {
            FieldPose targetPose = FieldPose.getClosestPose(m_swerveDrive.getPose(), poses);
            return Rotation2d.fromDegrees(m_launcher.getYawForTarget(m_swerveDrive, targetPose.getCurrentAlliancePose(), FieldDimensions.HubHeight).in(Degrees));
        }
    );
  }

  private Command getAimForwardCommand() {
    return DriveCommands.joystickDriveAtAngle(
        m_swerveDrive,
        DriverStation.isAutonomous() ? () -> 0 :  m_getDriverXTranslation,
        DriverStation.isAutonomous() ? () -> 0 : m_getDriverYTranslation,
        DriveDirection.Away::getAllianceAngle
    );
  }

  private boolean isAimedAtPose(FieldPose2026... poses) {
    Angle currentAngle = m_swerveDrive.getPose().getRotation().getMeasure();
    FieldPose targetPose = FieldPose.getClosestPose(m_swerveDrive.getPose(), poses);
    Angle targetAngle = m_launcher.getYawForTarget(m_swerveDrive, targetPose.getCurrentAlliancePose(), FieldDimensions.HubHeight);

    return targetAngle.isNear(currentAngle, LauncherConstants.AimYawTolerance.get()) 
      && m_getDriverXTranslation.getAsDouble() == 0 
      && m_getDriverYTranslation.getAsDouble() == 0;
  }

  private boolean isAimedAtAngle(Angle angle) {
    Angle currentAngle = m_swerveDrive.getPose().getRotation().getMeasure();
    

    return angle.isNear(currentAngle, LauncherConstants.AimYawTolerance.get()) 
      && m_getDriverXTranslation.getAsDouble() == 0 
      && m_getDriverYTranslation.getAsDouble() == 0;
  }

  private Command getAimWithXCommand(FieldPose2026... poses) {
    return new RepeatedConditionalCommand(
      new InstantCommand(m_swerveDrive::stopWithX, m_swerveDrive),
      getAimAtFieldPosesMovingCommand(poses), 
      () -> isAimedAtPose(poses)).repeatedly(); 
  }

  private Command getAimWithXCommand(Supplier<Angle> angle) {
    return new RepeatedConditionalCommand(
      new InstantCommand(m_swerveDrive::stopWithX, m_swerveDrive),
      getAimAtAngleCommand(angle), 
      () -> isAimedAtAngle(angle.get())).repeatedly(); 
  }


  private Command resetPoseCommand(DriveDirection direction) {
    return Commands.runOnce(
        () -> m_swerveDrive.setPose(
            new Pose2d(m_swerveDrive.getPose().getTranslation(), direction.getAllianceAngle())),
        m_swerveDrive)
        .ignoringDisable(true);
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

  public Intake getIntake() {
    return m_intake;
  }
}
