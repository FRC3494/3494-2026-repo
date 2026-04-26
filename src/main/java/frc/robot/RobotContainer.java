// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.util.QuadranglesUtil.*;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.autos.AutoBase;
import frc.robot.autos.Autos;
import frc.robot.autos.Autos.AutoRequirements;
import frc.robot.autos.DepotAndClimbAuto;
import frc.robot.autos.HubToDepotAuto;
import frc.robot.autos.HubToOutpostAuto;
import frc.robot.autos.LeftHubToClimbAuto;
import frc.robot.autos.LeftNZToClimbAuto;
import frc.robot.autos.LeftNZToNZAuto;
import frc.robot.autos.LeftNZWithPassingAuto;
import frc.robot.autos.RightClimbAuto;
import frc.robot.autos.RightHubToClimbAuto;
import frc.robot.autos.RightNZToClimbAuto;
import frc.robot.autos.RightNZToNZAuto;
import frc.robot.autos.RightOutpostAuto;
import frc.robot.autos.WarmUpAuto;
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
import frc.robot.util.choreo.ChoreoTraj;
import frc.robot.util.choreo.ChoreoVars;
import java.util.HashMap;
import java.util.function.Consumer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Sendable {
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
  private String selectedAutoNameCache = "";
  private boolean warmUpAutoSelected = false;
  private final AutoBase warmUpAuto;

  private Command selectedAutoCommand = none();
  private HashMap<String, Pose2d> autoStartingPoses = new HashMap<String, Pose2d>();

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

    warmUpAuto = new WarmUpAuto();

    configureCompetitionAutos();

    if (tuningMode) {
      configureTuningAutos();
    }

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("Buttons", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty(
        "Auto Warmup",
        () -> warmUpAutoSelected,
        (boolean value) -> {
          warmUpAutoSelected = value;

          if (value) {
            selectedAutoNameCache = autoChooser.selectedCommand().getName();
            autoChooser.select(
                warmUpAuto.getName() + (alliance == Alliance.Blue ? "_BLUE" : "_RED"));
          } else {
            autoChooser.select(selectedAutoNameCache);
          }
        });
  }

  // #region COMP AUTOS
  private void configureCompetitionAutos() {
    CommandScheduler.getInstance().schedule(autoFactory.warmupCmd());

    // Pre-load every trajectory into the AutoFactory's cache so the first auto run
    // doesn't pay the JSON-parse cost at enable time.
    for (ChoreoTraj traj : ChoreoTraj.ALL_TRAJECTORIES.values()) {
      Choreo.loadTrajectory(traj.name());

      if (traj.segment().isPresent()) {
        autoFactory.cache().loadTrajectory(traj.name(), traj.segment().getAsInt());
      } else {
        autoFactory.cache().loadTrajectory(traj.name());
      }
    }

    // Set up autos
    AutoBase[] autos =
        new AutoBase[] {
          new DepotAndClimbAuto(),
          new HubToDepotAuto(),
          new HubToOutpostAuto(),
          new LeftHubToClimbAuto(),
          new LeftNZToClimbAuto(),
          new LeftNZToNZAuto(),
          new LeftNZWithPassingAuto(),
          new RightClimbAuto(),
          new RightHubToClimbAuto(),
          new RightNZToClimbAuto(),
          new RightNZToNZAuto(),
          new RightOutpostAuto(),
          new WarmUpAuto()
        };

    AutoRequirements autoRequirements =
        new AutoRequirements(autoFactory, robotCommands, drive, shooterAimModel);

    for (AutoBase auto : autos) {
      autoChooser.addRoutine(
          auto.getName() + "_BLUE",
          () -> auto.getRoutine(auto.getName() + "_BLUE", Alliance.Blue, autoRequirements));
      autoChooser.addRoutine(
          auto.getName() + "_RED",
          () -> auto.getRoutine(auto.getName() + "_RED", Alliance.Red, autoRequirements));

      autoStartingPoses.put(auto.getName() + "_BLUE", auto.getStartingPose());
      autoStartingPoses.put(auto.getName() + "_RED", auto.getStartingPose());
    }

    autoChooser.addCmd("=====================", () -> none());

    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putData(
        "Buttons/ResetOdoForAuto",
        runOnce(
                () -> {
                  if (warmUpAutoSelected) {
                    drive.setPose(toAlliancePose(ChoreoVars.Poses.WarmUpPosition));
                  } else {
                    drive.setPose(
                        toAlliancePose(
                            autoStartingPoses.getOrDefault(
                                autoChooser.selectedCommand().getName(), Pose2d.kZero)));
                  }
                })
            .ignoringDisable(true));

    RobotModeTriggers.autonomous()
        .whileTrue(
            either(
                warmUpAuto.getRoutine(selectedAutoNameCache, alliance, autoRequirements).cmd(),
                autoChooser.selectedCommandScheduler(),
                () -> warmUpAutoSelected));

    RobotModeTriggers.teleop().onTrue(robotCommands.stopShootNoDelay());
  }
  // #endregion

  // #region TUNING AUTOS
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

    registerSysIdAutos(
        "Drive",
        drive.getDriveSysId(),
        (Voltage volts) -> drive.runDriveCharacterization(volts.in(Volts)));
    registerSysIdAutos(
        "Module Turn",
        drive.getTurnSysId(),
        (Voltage volts) -> drive.runTurnCharacterization(volts.in(Volts)));
    registerSysIdAutos(
        "Robot Turn",
        drive.getRobotTurnSysId(),
        (Voltage volts) -> drive.runRobotTurnCharacterization(volts.in(Volts)));

    autoChooser.addCmd(
        "Pigeon Turn Error Characterization", () -> DriveCommands.turnErrorCharacterization(drive));

    // Flywheel
    registerSysIdAutos("Flywheel", flywheel.getSysId(), flywheel::setOpenLoop);

    // Hopper
    registerSysIdAutos("Spindexer", hopper.getSpindexerSysId(), hopper::setSpindexerOpenLoop);
    registerSysIdAutos("Kicker", hopper.getKickerSysId(), hopper::setKickerOpenLoop);

    // Intake
    registerSysIdAutos(
        "Intake SpinnySpinny", intake.getSpinnySpinnySysId(), intake::setSpinnySpinnyOpenLoop);

    // Turret
    registerSysIdAutos("Turret", turret.getSysId(), turret::setOpenLoop);

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void registerSysIdAutos(
      String name, SysIdRoutine routine, Consumer<Voltage> setOpenLoop) {
    Command stopMechanism = run(() -> setOpenLoop.accept(Volts.of(0.0))).withTimeout(1.0);

    autoChooser.addCmd(
        name + " SysId (Quasistatic Forward)",
        () -> stopMechanism.andThen(routine.quasistatic(SysIdRoutine.Direction.kForward)));
    autoChooser.addCmd(
        name + " SysId (Quasistatic Reverse)",
        () -> stopMechanism.andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse)));

    autoChooser.addCmd(
        name + " SysId (Dynamic Forward)",
        () -> stopMechanism.andThen(routine.dynamic(SysIdRoutine.Direction.kForward)));
    autoChooser.addCmd(
        name + " SysId (Dynamic Reverse)",
        () -> stopMechanism.andThen(routine.dynamic(SysIdRoutine.Direction.kReverse)));
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

    SmartDashboard.putData("Buttons/ResetOdoLeftTrench", Autos.resetOdoLeftTrench(drive));
    SmartDashboard.putData("Buttons/ResetOdoRightTrench", Autos.resetOdoRightTrench(drive));
    SmartDashboard.putData("Buttons/ResetOdoLeftBump", Autos.resetOdoLeftBump(drive));
    SmartDashboard.putData("Buttons/ResetOdoRightBump", Autos.resetOdoRightBump(drive));

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
        .onFalse(
            either(
                either(
                    robotCommands.shootWoIntakeJostle(),
                    robotCommands.shoot(),
                    () -> IntakeOI.intake().getAsBoolean() || !shooterAimModel.isInAllianceZone()),
                robotCommands.stopSpindexer(),
                ShooterOI.shoot()::getAsBoolean));

    HopperOI.runKicker().onTrue(robotCommands.startKicker()).onFalse(robotCommands.stopKicker());

    HopperOI.kickerBackwards()
        .onTrue(robotCommands.startKickerReverse())
        .onFalse(
            either(
                either(
                    robotCommands.shootWoIntakeJostle(),
                    robotCommands.shoot(),
                    () -> IntakeOI.intake().getAsBoolean() || !shooterAimModel.isInAllianceZone()),
                robotCommands.stopKicker(),
                ShooterOI.shoot()::getAsBoolean));

    HopperOI.jiggleRobot().whileTrue(robotCommands.jiggleRobot());

    HopperOI.dumpFuel()
        .whileTrue(robotCommands.dumpFuelNoSafety())
        .onFalse(robotCommands.spinDownFromShoot());

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
                    robotCommands
                        .runIntakeJostleWithTrenchSafety()
                        .onlyIf(shooterAimModel::isInAllianceZone),
                    robotCommands.spinDownFromIntake(),
                    ShooterOI.shoot()::getAsBoolean)
                .withName("IntakeButtonRelease"));
    IntakeOI.intakeReverse()
        .onTrue(
            either(
                robotCommands.shootWithOuttakeWoJostle(),
                robotCommands.startIntakeReverse(),
                ShooterOI.shoot()::getAsBoolean))
        .onFalse(
            either(
                robotCommands.shoot(),
                robotCommands.stopIntake(),
                ShooterOI.shoot()::getAsBoolean));

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
                robotCommands.enableAutoTurret(),
                robotCommands.stopHood(),
                robotCommands.stopFlywheel()));

    ShooterOI.shoot()
        .whileTrue(
            either(
                    robotCommands.shootWithOuttakeWoJostle(),
                    either(
                        robotCommands.shootWoIntakeJostle(),
                        robotCommands.shoot(),
                        () ->
                            IntakeOI.intake().getAsBoolean()
                                || !shooterAimModel.isInAllianceZone()),
                    IntakeOI.intakeReverse()::getAsBoolean)
                .withName("ShootButtonPress"))
        .onFalse(
            sequence(
                    robotCommands.spinDownFromShoot(),
                    // startIntakeReverse() is instant, so it must go before intake(), which lasts
                    // for as long as you let it
                    robotCommands
                        .startIntakeReverse()
                        .onlyIf(IntakeOI.intakeReverse()::getAsBoolean),
                    robotCommands.intake().onlyIf(IntakeOI.intake()::getAsBoolean))
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
                    () -> alliance == Alliance.Blue)
                .withName("TrimRight"));
    ShooterOI.trimLeft()
        .whileTrue(
            either(
                    robotCommands.changeYTrim(true),
                    robotCommands.changeYTrim(false),
                    () -> alliance == Alliance.Blue)
                .withName("TrimLeft"));
    ShooterOI.trimForward()
        .whileTrue(
            either(
                    robotCommands.changeXTrim(true),
                    robotCommands.changeXTrim(false),
                    () -> alliance == Alliance.Blue)
                .withName("TrimForward"));
    ShooterOI.trimBack()
        .whileTrue(
            either(
                    robotCommands.changeXTrim(false),
                    robotCommands.changeXTrim(true),
                    () -> alliance == Alliance.Blue)
                .withName("TrimBack"));
    ShooterOI.resetXYTrim().onTrue(robotCommands.resetXYTrim());

    // #endregion

    // #region FLYWHEEL

    flywheel.setDefaultCommand(robotCommands.autoFlywheelCommand());

    FlywheelOI.startFlywheel().onTrue(robotCommands.startFlywheel());
    FlywheelOI.stopFlywheel().onTrue(robotCommands.stopFlywheel());

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

    // Commenting out default command so turret holds position via Turret.periodic() until
    // enableAutoTurret() is called (in auto after resetOdometry, or on teleop start).
    // turret.setDefaultCommand(robotCommands.autoTurretCommand());

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
    return selectedAutoCommand;
  }
}
