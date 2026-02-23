// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants.AutoAlignConstants;
import frc.robot.Constants.ElasticTab;
import frc.robot.OI.ClimberOI;
import frc.robot.OI.DriveOI;
import frc.robot.OI.IntakeOI;
import frc.robot.OI.ShooterOI;
import frc.robot.autos.Autos;
import frc.robot.autos.TestAuto;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.AimShooterCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.util.Elastic;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AprilTagVision aprilTagVision;
  private final Climber climber;
  private final Intake intake;
  private final Shooter shooter;

  // Choreo
  private final AutoChooser autoChooser;
  private final AutoFactory autoFactory;

  private final Command joystickDriveCommand;
  private final Command aimShooterCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
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
        break;
    }
    joystickDriveCommand =
        DriveCommands.joystickDrive(
            drive,
            OI.DriveOI::joystickDriveX,
            OI.DriveOI::joystickDriveY,
            OI.DriveOI::joystickDriveOmega);

    aprilTagVision = new AprilTagVision(drive);
    climber = new Climber();
    intake = new Intake();
    shooter = new Shooter();

    aimShooterCommand = new AimShooterCommand(shooter, drive);

    RobotModeTriggers.autonomous()
        .onTrue(runOnce(() -> Elastic.selectTab(ElasticTab.Autonomous.toString())));
    RobotModeTriggers.teleop()
        .onTrue(runOnce(() -> Elastic.selectTab(ElasticTab.Teleoperated.toString())));

    // Set up auto routines
    autoChooser = new AutoChooser();
    // TODO: add another argument at the end for TrajectoryLogger
    autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            true,
            drive,
            Autos::logTrajectory);
    configureAutos();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureAutos() {
    // Set up autos
    autoChooser.addRoutine("TestAuto", () -> TestAuto.getRoutine("TestAuto", autoFactory, drive));

    autoChooser.addCmd("=====================", () -> none());

    // Set up SysId routines
    // if (enableTuningAutos.get()) {
    if (true) {
      configureTuningAutos();
    } else {
      new Trigger(enableTuningAutos::get).onTrue(runOnce(this::configureTuningAutos));
    }

    SmartDashboard.putData("Auto Chooser", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  private void configureTuningAutos() {
    autoChooser.addCmd(
        "Drive Wheel Radius Rotational Characterization",
        () -> DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addCmd(
        "Drive Wheel Radius Linear Characterization",
        () -> DriveCommands.linearWheelRadiusCharacterization(drive));
    autoChooser.addCmd(
        "Drive Simple FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Forward)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Reverse)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Forward)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Reverse)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Pigeon Turn Error Characterization", () -> DriveCommands.turnErrorCharacterization(drive));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ==================== DRIVE ====================
    // Default command, normal field-relative drive
    drive.setDefaultCommand(joystickDriveCommand);

    DriveOI.autoAlignClimb()
        .onTrue(
            runOnce(
                () ->
                    drive.setDefaultCommand(
                        new AutoAlignCommand(AutoAlignConstants.climbPose, drive)),
                drive))
        .onFalse(runOnce(() -> drive.setDefaultCommand(joystickDriveCommand), drive));

    // Lock to 0° when A button is held
    DriveOI.lockToForward()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                OI.DriveOI::joystickDriveX,
                OI.DriveOI::joystickDriveY,
                () -> Rotation2d.kPi.div(4)));

    // Switch to X pattern when X button is pressed
    DriveOI.stopWithX().onTrue(runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Back button is pressed
    DriveOI.resetYaw().onTrue(runOnce(drive::resetYaw).ignoringDisable(true));

    // Rezero swerve turn relative encoders off of absolute encoders
    DriveOI.rezeroSwerveTurnEncoders()
        .onTrue(runOnce(drive::rezeroTurnEncoders).ignoringDisable(true));

    DriveOI.resetYawPigeon().onTrue(runOnce(drive::resetYawPigeon).ignoringDisable(true));

    // ==================== INTAKE ====================
    IntakeOI.intake()
        .onTrue(
            runOnce(
                () -> {
                  intake.setSpinnySpinnyVelocity(RPM.of(5.0));
                },
                intake))
        .onFalse(
            runOnce(
                () -> {
                  intake.setSpinnySpinnyVelocity(RPM.of(0.0));
                },
                intake));

    ShooterOI.runSpindexer()
        .onTrue(
            runOnce(
                () -> {
                  shooter.setSpindexerVelocity(RPM.of(3.0));
                },
                shooter))
        .onFalse(
            runOnce(
                () -> {
                  shooter.setSpindexerVelocity(RPM.of(0.0));
                },
                shooter));

    ShooterOI.runFeeder()
        .onTrue(
            runOnce(
                () -> {
                  shooter.setFeederVelocity(RPM.of(4.0));
                },
                shooter))
        .onFalse(
            runOnce(
                () -> {
                  shooter.setFeederVelocity(RPM.of(0.0));
                },
                shooter));

    ShooterOI.runHood()
        .onTrue(
            runOnce(
                () -> {
                  shooter.setPosition(RPM.of(0), Rotation2d.fromRadians(5.0), Rotation2d.kZero);
                },
                shooter))
        .onFalse(
            runOnce(
                () -> {
                  shooter.setPosition(RPM.of(0), Rotation2d.kZero, Rotation2d.kZero);
                },
                shooter));

    ShooterOI.runFlywheel()
        .onTrue(
            runOnce(
                () -> {
                  shooter.setPosition(RPM.of(0.5), Rotation2d.kZero, Rotation2d.kZero);
                },
                shooter))
        .onFalse(
            runOnce(
                () -> {
                  shooter.setPosition(RPM.of(0), Rotation2d.kZero, Rotation2d.kZero);
                },
                shooter));

    ClimberOI.runClimber()
        .onTrue(
            runOnce(
                () -> {
                  climber.setPosition(Rotation2d.fromRotations(3));
                },
                climber))
        .onFalse(
            runOnce(
                () -> {
                  climber.setPosition(Rotation2d.kZero);
                },
                climber));

    // ==================== SHOOTER ====================
    // shooter.setDefaultCommand(aimShooterCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
