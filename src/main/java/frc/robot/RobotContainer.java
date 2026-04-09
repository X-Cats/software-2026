// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.HoodKicker.HoodKicker;
import frc.robot.subsystems.HoodKicker.HoodKickerIOSim;
import frc.robot.subsystems.HoodKicker.HoodKickerIOTalonFX;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorIOSim;
import frc.robot.subsystems.conveyor.ConveyorIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoManager;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Conveyor conveyor;
  private final Intake intake;
  private final Shooter shooter;
  private final HoodKicker hood;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Robot State
  RobotStateMachine robotState = new RobotStateMachine();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    System.out.println(Constants.currentMode);
    System.out.println(Constants.simMode);
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        conveyor = new Conveyor(new ConveyorIOTalonFX(), robotState);
        intake = new Intake(new IntakeIOTalonFX(), robotState);
        shooter = new Shooter(new ShooterIOTalonFX(), robotState);
        hood = new HoodKicker(new HoodKickerIOTalonFX(), robotState);

        Camera cam = CameraConstants.RobotCameras.SHOOTER;

        vision = new Vision(cam);

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
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        conveyor = new Conveyor(new ConveyorIOSim(), robotState);
        intake = new Intake(new IntakeIOSim(), robotState);
        shooter = new Shooter(new ShooterIOSim(), robotState);
        hood = new HoodKicker(new HoodKickerIOSim(), robotState);

        vision = new Vision();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        conveyor = new Conveyor(new ConveyorIOSim(), robotState);
        intake = new Intake(new IntakeIOSim(), robotState);
        shooter = new Shooter(new ShooterIOSim(), robotState);
        hood = new HoodKicker(new HoodKickerIOSim(), robotState);
        vision = new Vision();
        break;
    }

    configureNamedCommands();
    var autoManager = new AutoManager();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", autoManager.getChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Do Nothing", new InstantCommand());

    // default bindings
    configureDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();
  }

  private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(Constants.DRIVE_SLEW_RATE);
  private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(Constants.DRIVE_SLEW_RATE);
  private final SlewRateLimiter thetaRateLimiter = new SlewRateLimiter(Constants.DRIVE_SLEW_RATE);
  /** */
  private void configureDefaultCommands() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> yRateLimiter.calculate(-controller.getLeftY()),
            () -> xRateLimiter.calculate(-controller.getLeftX()),
            () -> -thetaRateLimiter.calculate(controller.getRightX() * 2 / 3)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> yRateLimiter.calculate(-controller.getLeftY()),
                () -> xRateLimiter.calculate(-controller.getLeftX()),
                this::getHubDriveAngle));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.k180deg)),
                    drive)
                .ignoringDisable(true));

    // TODO bindings are not final
    // controller.leftTrigger().whileTrue(hopper.runConveyorMotor());
    // controller.button(0).whileTrue(hopper.runConveyorMotor());

    controller
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                    () -> {
                      robotState.setDesiredSuperState(RobotStateConfig.SuperState.SHOOTING);
                    },
                    () -> {
                      robotState.setDesiredSuperState(RobotStateConfig.SuperState.IDLE);
                    })
                .ignoringDisable(true));

    controller
        .leftBumper()
        .whileTrue(
            Commands.runEnd(
                    () -> {
                      robotState.setDesiredSuperState(RobotStateConfig.SuperState.INTAKING);
                    },
                    () -> {
                      robotState.setDesiredSuperState(RobotStateConfig.SuperState.IDLE);
                    })
                .ignoringDisable(true));

    controller
        .y()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  robotState.setDesiredSuperState(RobotStateConfig.SuperState.AGITATING);
                },
                () -> {
                  robotState.goBack();
                }));
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand(
        "Activate Intake",
        Commands.runEnd(
            () -> {
              robotState.setDesiredSuperState(RobotStateConfig.SuperState.INTAKING);
            },
            () -> {
              robotState.setDesiredSuperState(RobotStateConfig.SuperState.IDLE);
            }));
    NamedCommands.registerCommand(
        "Activate Shooting",
        Commands.runEnd(
            () -> {
              robotState.setDesiredSuperState(RobotStateConfig.SuperState.SHOOTING);
            },
            () -> {
              robotState.setDesiredSuperState(RobotStateConfig.SuperState.IDLE);
            }));

    NamedCommands.registerCommand(
        "Aim",
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            this::getHubDriveAngle));
  }

  private Rotation2d getHubDriveAngle() {
    Rotation2d hubAngle =
        AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d())
            .minus(RobotState.getInstance().getRobotPoseField().getTranslation())
            .getAngle();
    SmartDashboard.putNumber("Hub Drive Angle", hubAngle.getDegrees());
    return hubAngle;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
