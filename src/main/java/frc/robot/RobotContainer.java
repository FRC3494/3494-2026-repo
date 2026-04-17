// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ShooterConstants.FlywheelConstants.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
import frc.robot.OI.WonAutoState;
import frc.robot.autos.Autos;
import frc.robot.autos.DepotAndClimbAuto;
import frc.robot.autos.LeftNZToClimbAuto;
import frc.robot.autos.LeftNZToNZAuto;
import frc.robot.autos.RightClimbAuto;
import frc.robot.autos.RightNZToClimbAuto;
import frc.robot.autos.RightNZToNZAuto;
import frc.robot.autos.RightOutpostAuto;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.autoalign.AutoAlignToTargetCommands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.AimShooterMathLinear;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.util.Elastic;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final OI oi;

  // Subsystems
  private final Drive drive;
  private final AprilTagVision aprilTagVision;
  private final Climber climber;
  private final Hopper hopper;
  private final Intake intake;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;

  private final ShooterAimModel shooterAimModel;

  // Choreo
  private final AutoChooser autoChooser;
  private final AutoFactory autoFactory;

  public final RobotCommands robotCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    oi = new OI();

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

    // ! Hot swap shooter math classes here
    shooterAimModel = new AimShooterMathLinear(drive::getPose, drive::getChassisSpeeds);

    robotCommands =
        new RobotCommands(climber, drive, hopper, intake, flywheel, hood, turret, shooterAimModel);

    RobotModeTriggers.autonomous()
        .onTrue(
            runOnce(() -> Elastic.selectTab(ElasticTab.Autonomous.toString()))
                .ignoringDisable(true));

    RobotModeTriggers.autonomous()
        .onFalse(
            runOnce(() -> Elastic.selectTab(ElasticTab.Teleoperated.toString()))
                .ignoringDisable(true));

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
    configureCompetitionAutos();

    // Configure the button bindings
    configureButtonBindings();
  }

  // #region AUTOS
  private void configureCompetitionAutos() {
    // Set up autos
    autoChooser.addRoutine(
        "DepotAndClimb_BLUE",
        () ->
            DepotAndClimbAuto.getRoutine(
                "DepotAndClimb_BLUE",
                Alliance.Blue,
                autoFactory,
                robotCommands,
                drive,
                shooterAimModel));
    autoChooser.addRoutine(
        "DepotAndClimb_RED",
        () ->
            DepotAndClimbAuto.getRoutine(
                "DepotAndClimb_RED",
                Alliance.Red,
                autoFactory,
                robotCommands,
                drive,
                shooterAimModel));

    autoChooser.addRoutine(
        "LeftNZToClimb_BLUE",
        () ->
            LeftNZToClimbAuto.getRoutine(
                "LeftNZToClimb_BLUE",
                Alliance.Blue,
                autoFactory,
                robotCommands,
                drive,
                shooterAimModel));
    autoChooser.addRoutine(
        "LeftNZToClimb_RED",
        () ->
            LeftNZToClimbAuto.getRoutine(
                "LeftNZToClimb_RED",
                Alliance.Red,
                autoFactory,
                robotCommands,
                drive,
                shooterAimModel));
    autoChooser.addRoutine(
        "RightNZToClimb_BLUE",
        () ->
            RightNZToClimbAuto.getRoutine(
                "RightNZToClimb_BLUE",
                Alliance.Blue,
                autoFactory,
                robotCommands,
                drive,
                shooterAimModel));
    autoChooser.addRoutine(
        "RightNZToClimb_RED",
        () ->
            RightNZToClimbAuto.getRoutine(
                "RightNZToClimb_RED",
                Alliance.Red,
                autoFactory,
                robotCommands,
                drive,
                shooterAimModel));

    autoChooser.addRoutine(
        "LeftNZToNZ_BLUE",
        () ->
            LeftNZToNZAuto.getRoutine(
                "LeftNZToNZ_BLUE", Alliance.Blue, autoFactory, robotCommands, drive));
    autoChooser.addRoutine(
        "LeftNZToNZ_RED",
        () ->
            LeftNZToNZAuto.getRoutine(
                "LeftNZToNZ_RED", Alliance.Red, autoFactory, robotCommands, drive));
    autoChooser.addRoutine(
        "RightNZToNZ_BLUE",
        () ->
            RightNZToNZAuto.getRoutine(
                "RightNZToNZ_BLUE", Alliance.Blue, autoFactory, robotCommands, drive));
    autoChooser.addRoutine(
        "RightNZToNZ_RED",
        () ->
            RightNZToNZAuto.getRoutine(
                "RightNZToNZ_RED", Alliance.Red, autoFactory, robotCommands, drive));

    autoChooser.addRoutine(
        "RightClimb_BLUE",
        () ->
            RightClimbAuto.getRoutine(
                "RightClimb_BLUE", Alliance.Blue, autoFactory, robotCommands, drive));
    autoChooser.addRoutine(
        "RightClimb_RED",
        () ->
            RightClimbAuto.getRoutine(
                "RightClimb_RED", Alliance.Red, autoFactory, robotCommands, drive));

    autoChooser.addRoutine(
        "RightOutpost_BLUE",
        () ->
            RightOutpostAuto.getRoutine(
                "RightOutpost_BLUE",
                Alliance.Blue,
                autoFactory,
                robotCommands,
                drive,
                shooterAimModel));
    autoChooser.addRoutine(
        "RightOutpost_RED",
        () ->
            RightOutpostAuto.getRoutine(
                "RightOutpost_RED",
                Alliance.Red,
                autoFactory,
                robotCommands,
                drive,
                shooterAimModel));

    autoChooser.addCmd("=====================", () -> none());

    configureTuningAutos();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    RobotModeTriggers.teleop().onTrue(robotCommands.spinDownFromShoot());
  }

  private void configureTuningAutos() {
    // Drive
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
        () -> drive.driveSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Reverse)",
        () -> drive.driveSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Forward)",
        () -> drive.driveSysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Reverse)",
        () -> drive.driveSysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addCmd(
        "Turn SysId (Quasistatic Forward)",
        () -> drive.turnSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Turn SysId (Quasistatic Reverse)",
        () -> drive.turnSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Turn SysId (Dynamic Forward)",
        () -> drive.turnSysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Turn SysId (Dynamic Reverse)",
        () -> drive.turnSysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addCmd(
        "Robot Turn SysId (Quasistatic Forward)",
        () -> drive.robotTurnSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Robot Turn SysId (Quasistatic Reverse)",
        () -> drive.robotTurnSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Robot Turn SysId (Dynamic Forward)",
        () -> drive.robotTurnSysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Robot Turn SysId (Dynamic Reverse)",
        () -> drive.robotTurnSysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addCmd(
        "Pigeon Turn Error Characterization", () -> DriveCommands.turnErrorCharacterization(drive));

    // Flywheel
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

    // Hopper
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

    // Intake
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

    // Turret
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
  // #endregion

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // #region WHOLE ROBOT

    RobotModeTriggers.teleop().onTrue(runOnce(() -> OI.setWonAutoState(WonAutoState.Unknown)));
    OI.RumbleOI.shiftRumbleWindow().whileTrue(OI.RumbleOI.shiftRumbleSequence());

    SmartDashboard.putData("ResetOdoLeftTrench", Autos.resetOdoLeftTrench(drive));
    SmartDashboard.putData("ResetOdoRightTrench", Autos.resetOdoRightTrench(drive));
    SmartDashboard.putData("ResetOdoLeftBump", Autos.resetOdoLeftBump(drive));
    SmartDashboard.putData("ResetOdoRightBump", Autos.resetOdoRightBump(drive));

    // #endregion

    // #region CLIMBER

    ClimberOI.climberUp().onTrue(robotCommands.runClimberUp());
    ClimberOI.climberDown().onTrue(robotCommands.runClimberDown());
    ClimberOI.actuallyClimb().onTrue(robotCommands.runClimberMidWithCurrent());

    ClimberOI.rezeroClimber().onTrue(robotCommands.rezeroClimber());

    ClimberOI.climberManualUp().whileTrue(robotCommands.climberManualUp());
    ClimberOI.climberManualDown().whileTrue(robotCommands.climberManualDown());

    // #endregion

    // #region DRIVE

    drive.setDefaultCommand(robotCommands.joystickDriveCommand);

    DriveOI.resetYaw().onTrue(runOnce(drive::resetYaw).ignoringDisable(true).withName("ResetYaw"));
    DriveOI.rezeroSwerveTurnEncoders()
        .onTrue(
            runOnce(drive::rezeroTurnEncoders).ignoringDisable(true).withName("RezeroSwerveTurn"));

    DriveOI.stopWithX().onTrue(runOnce(drive::stopWithX, drive).withName("StopWithX"));

    DriveOI.autoAlignClimb()
        .whileTrue(AutoAlignToTargetCommands.autoAlignToTower(drive, robotCommands));
    DriveOI.autoDriveThroughTrench()
        .whileTrue(AutoAlignToTargetCommands.autoDriveThroughTrench(drive, robotCommands));

    // #endregion

    // #region HOPPER

    HopperOI.spindexerBackwards()
        .onTrue(robotCommands.startSpindexerReverse())
        .onFalse(robotCommands.stopSpindexer());

    HopperOI.runKicker().onTrue(robotCommands.startKicker()).onFalse(robotCommands.stopKicker());

    HopperOI.kickerBackwards()
        .onTrue(robotCommands.startKickerReverse())
        .onFalse(robotCommands.stopKicker());

    HopperOI.jiggleRobot().whileTrue(robotCommands.jiggleRobot());

    // #endregion

    // #region INTAKE

    IntakeOI.intake()
        .onTrue(
            either(
                    sequence(robotCommands.stopIntakeJostle(), robotCommands.startIntake()),
                    robotCommands.intake(),
                    ShooterOI.shoot()::getAsBoolean)
                .withName("IntakeButtonPress"))
        .whileFalse(
            either(
                    robotCommands.runIntakeJostle().onlyIf(shooterAimModel::isInAllianceZone),
                    robotCommands.spinDownFromIntake(),
                    ShooterOI.shoot()::getAsBoolean)
                .withName("IntakeButtonRelease"));
    IntakeOI.intakeReverse()
        .onTrue(robotCommands.startIntakeReverse())
        .onFalse(robotCommands.stopIntake());

    IntakeOI.toggleIntake()
        .onTrue(
            either(
                    robotCommands.intake(),
                    robotCommands.spinDownFromIntake(),
                    () -> intake.getSpinnySpinnySetpoint().isEquivalent(RPM.zero()))
                .withName("ToggleIntake"));

    IntakeOI.raiseIntake().whileTrue(robotCommands.intakeManualUp());
    IntakeOI.lowerIntake().whileTrue(robotCommands.intakeManualDown());
    IntakeOI.jostleIntake()
        .whileTrue(robotCommands.runIntakeJostle())
        .onFalse(robotCommands.stopIntakeJostle());

    // #endregion

    // #region SHOOTER

    RobotModeTriggers.teleop()
        .onTrue(
            sequence(
                robotCommands.enableAutoShooterSettings(),
                robotCommands.stopHood(),
                robotCommands.stopFlywheel()));

    ShooterOI.shoot()
        .whileTrue(
            either(
                    robotCommands.shootWithoutIntakeJostle(),
                    robotCommands.shoot(),
                    () -> IntakeOI.intake().getAsBoolean() || !shooterAimModel.isInAllianceZone())
                .withName("ShootButtonPress"))
        .onFalse(
            either(
                    sequence(robotCommands.spinDownFromShoot(), robotCommands.intake()),
                    robotCommands.spinDownFromShoot(),
                    IntakeOI.intake()::getAsBoolean)
                .withName("ShootButtonRelease"));

    ShooterOI.shootClose()
        .whileTrue(
            sequence(robotCommands.setCloseShot(false), robotCommands.shootWithManualSettings())
                .withName("ShootClose"))
        .onFalse(robotCommands.manualShootRelease());
    ShooterOI.shootCloseWithTrim()
        .whileTrue(
            sequence(robotCommands.setCloseShot(true), robotCommands.shootWithManualSettings())
                .withName("ShootCloseWithTrim"))
        .onFalse(robotCommands.manualShootRelease());

    ShooterOI.shootMedium()
        .whileTrue(
            sequence(robotCommands.setMediumShot(false), robotCommands.shootWithManualSettings())
                .withName("ShootMedium"))
        .onFalse(robotCommands.manualShootRelease());
    ShooterOI.shootMediumWithTrim()
        .whileTrue(
            sequence(robotCommands.setMediumShot(true), robotCommands.shootWithManualSettings())
                .withName("ShootMediumWithTrim"))
        .onFalse(robotCommands.manualShootRelease());

    ShooterOI.shootFar()
        .whileTrue(
            sequence(robotCommands.setFarShot(false), robotCommands.shootWithManualSettings())
                .withName("ShootFar"))
        .onFalse(robotCommands.manualShootRelease());
    ShooterOI.shootFarWithTrim()
        .whileTrue(
            sequence(robotCommands.setFarShot(true), robotCommands.shootWithManualSettings())
                .withName("ShootFarWithTrim"))
        .onFalse(robotCommands.manualShootRelease());

    ShooterOI.shootNeutralZone()
        .whileTrue(
            sequence(
                    robotCommands.setNeutralZoneShot(false),
                    robotCommands.shootWithManualSettings())
                .withName("ShootNZ"))
        .onFalse(robotCommands.manualShootRelease());
    ShooterOI.shootNeutralZoneWithTrim()
        .whileTrue(
            sequence(
                    robotCommands.setNeutralZoneShot(true), robotCommands.shootWithManualSettings())
                .withName("ShootNZWithTrim"))
        .onFalse(robotCommands.manualShootRelease());

    ShooterOI.shootDashboard()
        .whileTrue(
            sequence(robotCommands.setDashboardShot(), robotCommands.shootWithManualSettings())
                .withName("ShootDashboard"))
        .onFalse(robotCommands.manualShootRelease());

    ShooterOI.increaseDistanceTrim().whileTrue(robotCommands.changeDistanceTrim(true));
    ShooterOI.decreaseDistanceTrim().whileTrue(robotCommands.changeDistanceTrim(false));
    ShooterOI.resetDistanceTrim().onTrue(robotCommands.resetDistanceTrim());

    ShooterOI.trimRight()
        .whileTrue(
            either(
                    robotCommands.changeYTrim(false),
                    robotCommands.changeYTrim(true),
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                .withName("TrimRight"));
    ShooterOI.trimLeft()
        .whileTrue(
            either(
                    robotCommands.changeYTrim(true),
                    robotCommands.changeYTrim(false),
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                .withName("TrimLeft"));
    ShooterOI.trimForward()
        .whileTrue(
            either(
                    robotCommands.changeXTrim(true),
                    robotCommands.changeXTrim(false),
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                .withName("TrimForward"));
    ShooterOI.trimBack()
        .whileTrue(
            either(
                    robotCommands.changeXTrim(false),
                    robotCommands.changeXTrim(true),
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                .withName("TrimBack"));
    ShooterOI.resetXYTrim().onTrue(robotCommands.resetXYTrim());

    // #endregion

    // #region FLYWHEEL

    flywheel.setDefaultCommand(robotCommands.autoFlywheelCommand());

    FlywheelOI.runFlywheel()
        .onTrue(robotCommands.setFlywheelManual(() -> flywheelManualSpeed))
        .onFalse(robotCommands.flywheelManualStop());

    FlywheelOI.increaseFlywheelTrim()
        .whileTrue(
            run(
                    () -> {
                      shooterAimModel.setFlywheelTrim(
                          shooterAimModel.getFlywheelTrim().plus(RPM.of(5)));
                    },
                    shooterAimModel)
                .withName("IncreaseFlywheelTrim"));
    FlywheelOI.decreaseFlywheelTrim()
        .whileTrue(
            run(
                    () -> {
                      shooterAimModel.setFlywheelTrim(
                          shooterAimModel.getFlywheelTrim().minus(RPM.of(5)));
                    },
                    shooterAimModel)
                .withName("DecreaseFlywheelTrim"));

    // #endregion

    // #region HOOD

    hood.setDefaultCommand(robotCommands.autoHoodCommand());

    HoodOI.rezeroHood().onTrue(robotCommands.rezeroHood());

    HoodOI.increaseHoodTrim()
        .whileTrue(
            run(
                    () -> {
                      shooterAimModel.setHoodTrim(
                          shooterAimModel.getHoodTrim().plus(Rotation2d.fromDegrees(0.5)));
                    },
                    shooterAimModel)
                .withName("IncreaseHoodTrim"));
    HoodOI.decreaseHoodTrim()
        .whileTrue(
            run(
                    () -> {
                      shooterAimModel.setHoodTrim(
                          shooterAimModel.getHoodTrim().minus(Rotation2d.fromDegrees(0.5)));
                    },
                    shooterAimModel)
                .withName("DecreaseHoodTrim"));

    HoodOI.hoodManualUp().whileTrue(robotCommands.runHoodManualUp());
    HoodOI.hoodManualDown().whileTrue(robotCommands.runHoodManualDown());

    // #endregion

    // #region TURRET

    turret.setDefaultCommand(robotCommands.autoTurretCommand());

    TurretOI.turretManualCCW().whileTrue(robotCommands.runTurretManualCCW());
    TurretOI.turretManualCW().whileTrue(robotCommands.runTurretManualCW());

    TurretOI.resetTurretTrim().onTrue(robotCommands.resetTurretTrim());
    TurretOI.increaseTurretTrim().onTrue(robotCommands.changeTurretTrim(true));
    TurretOI.decreaseTurretTrim().onTrue(robotCommands.changeTurretTrim(false));

    TurretOI.lockTurret().onTrue(robotCommands.lockTurret());
    TurretOI.enableAutoTurret().onTrue(robotCommands.enableAutoTurret());
    SmartDashboard.putData("Buttons/EnableAutoTurret", robotCommands.enableAutoTurret());

    TurretOI.rezeroTurret().onTrue(robotCommands.rezeroTurret());
    SmartDashboard.putData("Buttons/RezeroTurret", robotCommands.rezeroTurret());

    SmartDashboard.putData(
        "Buttons/TurretTo0", robotCommands.turretToPosition(Units.degreesToRotations(0)));
    SmartDashboard.putData(
        "Buttons/TurretTo90", robotCommands.turretToPosition(Units.degreesToRotations(90)));
    SmartDashboard.putData(
        "Buttons/TurretTo180", robotCommands.turretToPosition(Units.degreesToRotations(180)));
    SmartDashboard.putData(
        "Buttons/TurretTo270", robotCommands.turretToPosition(Units.degreesToRotations(270)));

    // #endregion
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
