// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ClimberConstants.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElasticTab;
import frc.robot.OI.ClimberOI;
import frc.robot.OI.DriveOI;
import frc.robot.OI.HopperOI;
import frc.robot.OI.IntakeOI;
import frc.robot.OI.ShooterOI;
import frc.robot.OI.ShooterOI.FlywheelOI;
import frc.robot.OI.ShooterOI.HoodOI;
import frc.robot.OI.ShooterOI.TurretOI;
import frc.robot.autos.Autos;
import frc.robot.autos.DepotAndClimbAuto;
import frc.robot.autos.RightClimbAuto;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.AimShooterMathKinematics;
import frc.robot.subsystems.shooter.AimShooterMathLinear;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.util.Elastic;
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

  private final AimShooterMathKinematics aimShooterMath;
  private final AimShooterMathLinear aimShooterMathLinear;
  private final ShooterAimModel shooterAimModel;

  // Choreo
  private final AutoChooser autoChooser;
  private final AutoFactory autoFactory;

  private final RobotCommands robotCommands;

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

    aprilTagVision = new AprilTagVision(drive);
    climber = new Climber();
    hopper = new Hopper();
    intake = new Intake();
    flywheel = new Flywheel();
    hood = new Hood();
    turret = new Turret();

    aimShooterMath = new AimShooterMathKinematics(drive::getPose, drive::getChassisSpeeds);
    aimShooterMathLinear = new AimShooterMathLinear(drive::getPose, drive::getChassisSpeeds);

    shooterAimModel = aimShooterMathLinear;

    robotCommands =
        new RobotCommands(climber, drive, hopper, intake, flywheel, hood, turret, shooterAimModel);
    RobotModeTriggers.autonomous()
        .onTrue(runOnce(() -> Elastic.selectTab(ElasticTab.Autonomous.toString())));
    RobotModeTriggers.teleop()
        .onTrue(runOnce(() -> Elastic.selectTab(ElasticTab.Teleoperated.toString())));

    // Set up auto routines
    autoChooser = new AutoChooser();
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
        "DepotAndClimb",
        () -> DepotAndClimbAuto.getRoutine("DepotAndClimb", autoFactory, robotCommands, drive));
    autoChooser.addRoutine(
        "RightClimb",
        () -> RightClimbAuto.getRoutine("RightClimb", autoFactory, robotCommands, drive));

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
    // ==================== Drive ====================
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

    // ==================== Flywheel ====================
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

    // ==================== Hopper ====================
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

    // ==================== Intake ====================
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

    // ==================== Turret ====================
    autoChooser.addCmd(
        "Turret SysId (Quasistatic Forward)",
        () -> turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Turret SysId (Quasistatic Reverse)",
        () -> turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Turret SysId (Dynamic Forward)",
        () -> turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Turret SysId (Dynamic Reverse)",
        () -> turret.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ==================== WHOLE ROBOT ====================
    OI.rezeroMechanisms().onTrue(robotCommands.rezeroMechanisms());

    // ==================== CLIMBER ====================
    ClimberOI.climberUp().onTrue(robotCommands.climberUp());

    ClimberOI.climberDown()
        .onTrue(
            either(
                robotCommands.climberMid(),
                robotCommands.climberDown(),
                () ->
                    climber.getPosition()
                        <= climberDownPosition * robotCommands.climberMidFactor.get() - 0.05));

    ClimberOI.rezeroClimber().onTrue(robotCommands.rezeroClimber());

    ClimberOI.climberManualUp()
        .onTrue(robotCommands.climberManualUp())
        .onFalse(robotCommands.stopClimber());
    ClimberOI.climberManualDown()
        .onTrue(robotCommands.climberManualDown())
        .onFalse(robotCommands.stopClimber());

    // ==================== DRIVE ====================
    drive.setDefaultCommand(robotCommands.joystickDriveCommand);

    DriveOI.resetYaw().onTrue(runOnce(drive::resetYaw).ignoringDisable(true));
    DriveOI.rezeroSwerveTurnEncoders()
        .onTrue(runOnce(drive::rezeroTurnEncoders).ignoringDisable(true));

    DriveOI.stopWithX().onTrue(runOnce(drive::stopWithX, drive));

    // ! Currently disabled
    DriveOI.lockTo45()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                OI.DriveOI::joystickDriveX,
                OI.DriveOI::joystickDriveY,
                () -> Rotation2d.kPi.div(4)));

    DriveOI.autoAlignClimb()
        .onTrue(robotCommands.autoAlignClimb())
        .onFalse(
            runOnce(
                () -> {
                  drive.setDefaultCommand(robotCommands.joystickDriveCommand);
                },
                drive));

    // ==================== HOPPER ====================
    HopperOI.runSpindexerBackwards()
        .onTrue(robotCommands.runSpindexerReverse())
        .onFalse(robotCommands.stopSpindexer());

    HopperOI.runKicker().onTrue(robotCommands.runKicker()).onFalse(robotCommands.stopKicker());

    HopperOI.jiggleRobot().whileTrue(robotCommands.jiggleRobot());

    // ==================== INTAKE ====================
    IntakeOI.intake().onTrue(robotCommands.intake()).onFalse(robotCommands.releaseIntake());
    IntakeOI.outtake().onTrue(robotCommands.runIntakeReverse()).onFalse(robotCommands.stopIntake());

    IntakeOI.toggleIntake()
        .onTrue(
            either(
                robotCommands.intake(),
                robotCommands.releaseIntake(),
                () -> intake.getSpinnySpinnySetpoint().isEquivalent(RPM.of(0))));

    IntakeOI.rezeroIntakeUppyDowny().onTrue(robotCommands.rezeroIntakeUppyDowny());

    // ==================== SHOOTER ====================
    RobotModeTriggers.teleop().onTrue(robotCommands.enableAutoShooterSettings());

    ShooterOI.shoot().whileTrue(robotCommands.shoot()).onFalse(robotCommands.spinDownFromShoot());

    ShooterOI.shootClose()
        .onTrue(sequence(robotCommands.setCloseShot(false), robotCommands.shoot()))
        .onFalse(
            sequence(robotCommands.spinDownFromShoot(), robotCommands.enableAutoShooterSettings()));
    ShooterOI.shootCloseWithTrim()
        .onTrue(sequence(robotCommands.setCloseShot(true), robotCommands.shoot()))
        .onFalse(
            sequence(robotCommands.spinDownFromShoot(), robotCommands.enableAutoShooterSettings()));

    ShooterOI.shootMedium()
        .onTrue(sequence(robotCommands.setMediumShot(false), robotCommands.shoot()))
        .onFalse(
            sequence(robotCommands.spinDownFromShoot(), robotCommands.enableAutoShooterSettings()));
    ShooterOI.shootMediumWithTrim()
        .onTrue(sequence(robotCommands.setMediumShot(true), robotCommands.shoot()))
        .onFalse(
            sequence(robotCommands.spinDownFromShoot(), robotCommands.enableAutoShooterSettings()));

    ShooterOI.shootFar()
        .onTrue(sequence(robotCommands.setFarShot(false), robotCommands.shoot()))
        .onFalse(
            sequence(robotCommands.spinDownFromShoot(), robotCommands.enableAutoShooterSettings()));
    ShooterOI.shootFarWithTrim()
        .onTrue(sequence(robotCommands.setFarShot(true), robotCommands.shoot()))
        .onFalse(
            sequence(robotCommands.spinDownFromShoot(), robotCommands.enableAutoShooterSettings()));

    ShooterOI.shootNeutralZone()
        .onTrue(sequence(robotCommands.setNeutralZoneShot(false), robotCommands.shoot()))
        .onFalse(
            sequence(robotCommands.spinDownFromShoot(), robotCommands.enableAutoShooterSettings()));
    ShooterOI.shootNeutralZoneWithTrim()
        .onTrue(sequence(robotCommands.setNeutralZoneShot(true), robotCommands.shoot()))
        .onFalse(
            sequence(robotCommands.spinDownFromShoot(), robotCommands.enableAutoShooterSettings()));

    ShooterOI.shootDashboard()
        .onTrue(sequence(robotCommands.setDashboardShot(), robotCommands.shoot()))
        .onFalse(
            sequence(robotCommands.spinDownFromShoot(), robotCommands.enableAutoShooterSettings()));

    ShooterOI.resetShooterTrims()
        .onTrue(
            runOnce(
                    () -> {
                      shooterAimModel.setDistanceTrim(Inches.of(0));
                      shooterAimModel.setFlywheelTrim(RPM.of(0));
                      shooterAimModel.setHoodTrim(Rotation2d.kZero);
                    },
                    shooterAimModel)
                .ignoringDisable(true));

    ShooterOI.increaseDistanceTrim()
        .whileTrue(
            run(
                () -> {
                  shooterAimModel.setDistanceTrim(
                      shooterAimModel.getDistanceTrim().plus(Inches.of(1)));
                },
                shooterAimModel));
    ShooterOI.decreaseDistanceTrim()
        .whileTrue(
            run(
                () -> {
                  shooterAimModel.setDistanceTrim(
                      shooterAimModel.getDistanceTrim().minus(Inches.of(1)));
                },
                shooterAimModel));
    ShooterOI.resetDistanceTrim()
        .onTrue(
            runOnce(
                () -> {
                  shooterAimModel.setDistanceTrim(Inches.of(0));
                },
                shooterAimModel));

    Distance xyTrimSensitivity = Inches.of(1.0);
    ShooterOI.trimRight()
        .whileTrue(
            run(
                () -> {
                  if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                    shooterAimModel.setYTrim(shooterAimModel.getYTrim().minus(xyTrimSensitivity));
                  } else {
                    shooterAimModel.setYTrim(shooterAimModel.getYTrim().plus(xyTrimSensitivity));
                  }
                },
                shooterAimModel));
    ShooterOI.trimLeft()
        .whileTrue(
            run(
                () -> {
                  if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                    shooterAimModel.setYTrim(shooterAimModel.getYTrim().plus(xyTrimSensitivity));
                  } else {
                    shooterAimModel.setYTrim(shooterAimModel.getYTrim().minus(xyTrimSensitivity));
                  }
                },
                shooterAimModel));
    ShooterOI.trimForward()
        .whileTrue(
            run(
                () -> {
                  if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                    shooterAimModel.setXTrim(shooterAimModel.getXTrim().plus(xyTrimSensitivity));
                  } else {
                    shooterAimModel.setXTrim(shooterAimModel.getXTrim().minus(xyTrimSensitivity));
                  }
                },
                shooterAimModel));
    ShooterOI.trimBack()
        .whileTrue(
            run(
                () -> {
                  if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                    shooterAimModel.setXTrim(shooterAimModel.getXTrim().minus(xyTrimSensitivity));
                  } else {
                    shooterAimModel.setXTrim(shooterAimModel.getXTrim().plus(xyTrimSensitivity));
                  }
                },
                shooterAimModel));

    ShooterOI.resetXYTrim()
        .onTrue(
            runOnce(
                () -> {
                  shooterAimModel.setXTrim(Inches.of(0));
                  shooterAimModel.setYTrim(Inches.of(0));
                },
                shooterAimModel));

    // ==================== FLYWHEEL ====================
    flywheel.setDefaultCommand(robotCommands.setFlywheelCommand);

    FlywheelOI.runFlywheel()
        .onTrue(robotCommands.runFlywheelManual(RPM.of(robotCommands.flywheelSpeed.get())))
        .onFalse(robotCommands.stopFlywheel());

    FlywheelOI.increaseFlywheelTrim()
        .whileTrue(
            run(
                () -> {
                  shooterAimModel.setFlywheelTrim(
                      shooterAimModel.getFlywheelTrim().plus(RPM.of(5)));
                },
                shooterAimModel));
    FlywheelOI.decreaseFlywheelTrim()
        .whileTrue(
            run(
                () -> {
                  shooterAimModel.setFlywheelTrim(
                      shooterAimModel.getFlywheelTrim().minus(RPM.of(5)));
                },
                shooterAimModel));

    // ==================== HOOD ====================
    hood.setDefaultCommand(robotCommands.setHoodCommand);

    HoodOI.rezeroHood().onTrue(robotCommands.rezeroHood());

    HoodOI.increaseHoodTrim()
        .whileTrue(
            run(
                () -> {
                  shooterAimModel.setHoodTrim(
                      shooterAimModel.getHoodTrim().plus(Rotation2d.fromDegrees(0.5)));
                },
                shooterAimModel));
    HoodOI.decreaseHoodTrim()
        .whileTrue(
            run(
                () -> {
                  shooterAimModel.setHoodTrim(
                      shooterAimModel.getHoodTrim().minus(Rotation2d.fromDegrees(0.5)));
                },
                shooterAimModel));

    HoodOI.hoodManualUp().whileTrue(robotCommands.hoodManualUp());
    HoodOI.hoodManualDown().whileTrue(robotCommands.hoodManualDown());

    // ==================== TURRET ====================
    TurretOI.turretManualCCW().whileTrue(robotCommands.turretManualCCW());
    TurretOI.turretManualCW().whileTrue(robotCommands.turretManualCW());

    TurretOI.resetTurretTrim()
        .onTrue(
            runOnce(
                    () -> {
                      shooterAimModel.setTurretTrim(Units.degreesToRotations(0.0));
                    },
                    shooterAimModel)
                .ignoringDisable(true));

    TurretOI.increaseTurretTrim()
        .whileTrue(
            run(
                () -> {
                  shooterAimModel.setTurretTrim(
                      shooterAimModel.getTurretTrimRot() + Units.degreesToRotations(1));
                },
                shooterAimModel));
    TurretOI.decreaseTurretTrim()
        .whileTrue(
            run(
                () -> {
                  shooterAimModel.setTurretTrim(
                      shooterAimModel.getTurretTrimRot() - Units.degreesToRotations(1));
                },
                shooterAimModel));

    TurretOI.rezeroTurret().onTrue(robotCommands.rezeroTurret());
    TurretOI.unmurderTurret().onTrue(robotCommands.unmurderTurret());
    TurretOI.lockTurret().onTrue(robotCommands.lockTurret());
    TurretOI.enableAutoTurret().onTrue(robotCommands.enableAutoTurret());
    SmartDashboard.putData("Buttons/EnableAutoTurret", robotCommands.enableAutoTurret());

    TurretOI.setTurretEncoderTo0().onTrue(robotCommands.setTurretEncoderTo0());

    TurretOI.turretTo180().onTrue(robotCommands.turretToPosition(Units.degreesToRotations(180)));
    TurretOI.turretTo90().onTrue(robotCommands.turretToPosition(Units.degreesToRotations(90)));
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
