// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ClimberConstants.climberMinPosition;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants.AutoAlignConstants;
import frc.robot.Constants.ElasticTab;
import frc.robot.OI.ClimberOI;
import frc.robot.OI.DriveOI;
import frc.robot.OI.IntakeOI;
import frc.robot.OI.ShooterOI;
import frc.robot.autos.Autos;
import frc.robot.autos.ClimbLeftAuto;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.RezeroClimberCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.AimShooterMath;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.RezeroHoodCommand;
import frc.robot.subsystems.shooter.turret.RezeroTurretCommand;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.util.Elastic;
import frc.robot.util.QuadranglesUtil;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

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
  private final Hopper hopper;
  private final Intake intake;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;

  // Choreo
  private final AutoChooser autoChooser;
  private final AutoFactory autoFactory;

  private final RobotCommands robotCommands;
  private final Command joystickDriveCommand;

  private final AimShooterMath aimShooterMath;

  private LoggedNetworkBoolean enableTuningAutos =
      new LoggedNetworkBoolean("SmartDashboard/EnableTuningAutos", true);

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
    hopper = new Hopper();
    intake = new Intake();
    flywheel = new Flywheel();
    hood = new Hood();
    turret = new Turret();

    robotCommands = new RobotCommands(climber, drive, hopper, intake, flywheel, hood, turret);
    aimShooterMath = new AimShooterMath(drive::getPose);

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
    autoChooser.addRoutine(
        "ClimbLeft",
        () -> ClimbLeftAuto.getRoutine("ClimbLeft", autoFactory, robotCommands, climber));

    autoChooser.addCmd("=====================", () -> none());

    // Set up SysId routines
    if (enableTuningAutos.get()) {
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

    autoChooser.addCmd(
        "Flywheel SysId (Quasistatic Forward)",
        () -> flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Flywheel SysId (Quasistatic Reverse)",
        () -> flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Flywheel SysId (Dynamic Forward)",
        () -> flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Flywheel SysId (Dynamic Reverse)",
        () -> flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addCmd(
        "Spindexer SysId (Quasistatic Forward)",
        () -> hopper.spindexerSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Spindexer SysId (Quasistatic Reverse)",
        () -> hopper.spindexerSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Spindexer SysId (Dynamic Forward)",
        () -> hopper.spindexerSysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Spindexer SysId (Dynamic Reverse)",
        () -> hopper.spindexerSysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addCmd(
        "Kicker SysId (Quasistatic Forward)",
        () -> hopper.kickerSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Kicker SysId (Quasistatic Reverse)",
        () -> hopper.kickerSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Kicker SysId (Dynamic Forward)",
        () -> hopper.kickerSysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Kicker SysId (Dynamic Reverse)",
        () -> hopper.kickerSysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addCmd(
        "Intake SpinnySpinny SysId (Quasistatic Forward)",
        () -> intake.spinnySpinnySysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Intake SpinnySpinny SysId (Quasistatic Reverse)",
        () -> intake.spinnySpinnySysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Intake SpinnySpinny SysId (Dynamic Forward)",
        () -> intake.spinnySpinnySysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Intake SpinnySpinny SysId (Dynamic Reverse)",
        () -> intake.spinnySpinnySysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ==================== CLIMBER ====================
    ClimberOI.climberUp().onTrue(robotCommands.climberUp());

    ClimberOI.climberDown()
        .onTrue(
            either(
                robotCommands.climberMid(),
                robotCommands.climberDown(),
                () -> climber.getPosition() <= climberMinPosition / 2.0 - 0.05));

    ClimberOI.rezeroClimber().onTrue(RezeroClimberCommand.getCommand(climber));

    ClimberOI.climberManualUp()
        .onTrue(robotCommands.climberManualUp())
        .onFalse(robotCommands.stopClimber());

    ClimberOI.climberManualDown()
        .onTrue(robotCommands.climberManualDown())
        .onFalse(robotCommands.stopClimber());

    // ==================== DRIVE ====================
    // Default command, normal field-relative drive
    drive.setDefaultCommand(joystickDriveCommand);

    DriveOI.autoAlignClimb()
        .onTrue(
            runOnce(
                () -> {
                  double distanceToOutpostPose =
                      drive
                          .getPose()
                          .getTranslation()
                          .getDistance(
                              QuadranglesUtil.toAllianceTranslation(
                                  AutoAlignConstants.climbSetupPoseOutpost.getTranslation()));
                  double distanceToDepotPose =
                      drive
                          .getPose()
                          .getTranslation()
                          .getDistance(
                              QuadranglesUtil.toAllianceTranslation(
                                  AutoAlignConstants.climbSetupPoseDepot.getTranslation()));

                  if (distanceToOutpostPose < distanceToDepotPose) {
                    drive.setDefaultCommand(
                        sequence(
                            new AutoAlignCommand(AutoAlignConstants.climbSetupPoseOutpost, drive),
                            new AutoAlignCommand(AutoAlignConstants.climbPoseOutpost, drive)));
                  } else {
                    drive.setDefaultCommand(
                        sequence(
                            new AutoAlignCommand(AutoAlignConstants.climbSetupPoseDepot, drive),
                            new AutoAlignCommand(AutoAlignConstants.climbPoseDepot, drive)));
                  }
                },
                drive))
        .onFalse(
            runOnce(
                () -> {
                  drive.setDefaultCommand(joystickDriveCommand);
                },
                drive));

    // Lock to 0° when A button is held
    DriveOI.lockTo45()
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

    // ==================== HOPPER ====================
    ShooterOI.runSpindexer()
        .onTrue(robotCommands.runSpindexerReverse())
        .onFalse(robotCommands.stopSpindexer());

    ShooterOI.runKicker().onTrue(robotCommands.runKicker()).onFalse(robotCommands.stopKicker());

    // ==================== INTAKE ====================
    IntakeOI.intake().onTrue(robotCommands.intake()).onFalse(robotCommands.releaseIntake());

    IntakeOI.outtake().onTrue(robotCommands.runIntakeReverse()).onFalse(robotCommands.stopIntake());

    // ==================== SHOOTER ====================
    // shooter.setDefaultCommand(aimShooterCommand);

    ShooterOI.setHubShot().onTrue(robotCommands.setHubShot());
    ShooterOI.setTrenchShot().onTrue(robotCommands.setTrenchShot());
    ShooterOI.setOutpostShot().onTrue(robotCommands.setOutpostShot());
    ShooterOI.setNeutralZoneShot().onTrue(robotCommands.setNeutralZoneShot());

    ShooterOI.shoot().onTrue(robotCommands.shoot()).onFalse(robotCommands.spinDownFromShoot());

    // ==================== FLYWHEEL ====================
    ShooterOI.runFlywheel()
        .onTrue(robotCommands.runFlywheel())
        .onFalse(robotCommands.stopFlywheel());

    // ==================== HOOD ====================
    ShooterOI.increaseHood().whileTrue(robotCommands.increaseHoodAngle());

    ShooterOI.decreaseHood().whileTrue(robotCommands.decreaseHoodAngle());

    ShooterOI.rezeroHood().onTrue(RezeroHoodCommand.getCommand(hood));

    // ==================== TURRET ====================
    ShooterOI.turretManualNegative()
        .onTrue(
            sequence(
                runOnce(
                    () -> {
                      turret.setOpenLoop(Volts.of(-1.5));
                    },
                    turret),
                waitUntil(() -> turret.getAbsPosition().getDegrees() <= 49.7),
                runOnce(
                    () -> {
                      turret.setOpenLoop(Volts.of(0));
                    },
                    turret)))
        .onFalse(
            runOnce(
                () -> {
                  turret.setOpenLoop(Volts.of(0));
                },
                turret));
    ShooterOI.turretManualPositive()
        .onTrue(
            sequence(
                runOnce(
                    () -> {
                      turret.setOpenLoop(Volts.of(1.5));
                    },
                    turret),
                waitUntil(() -> turret.getAbsPosition().getDegrees() >= 430),
                runOnce(
                    () -> {
                      turret.setOpenLoop(Volts.of(0));
                    },
                    turret)))
        .onFalse(
            runOnce(
                () -> {
                  turret.setOpenLoop(Volts.of(0));
                },
                turret));

    ShooterOI.turretTo180()
        .onTrue(
            runOnce(
                () -> {
                  turret.setPosition(Rotation2d.k180deg);
                },
                turret));
    ShooterOI.turretTo90()
        .onTrue(
            runOnce(
                () -> {
                  turret.setPosition(Rotation2d.kCCW_90deg);
                },
                turret));

    ShooterOI.setTurretEncoderTo0()
        .onTrue(
            runOnce(
                    () -> {
                      turret.setRelativeEncoderPosition(Rotation2d.kZero);
                    },
                    turret)
                .ignoringDisable(true));
    ShooterOI.rezeroTurret().onTrue(RezeroTurretCommand.getCommand(turret).ignoringDisable(true));
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
