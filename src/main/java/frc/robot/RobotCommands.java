package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.HopperConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.IntakeConstants.uppyDownyCurrentLimit;
import static frc.robot.Constants.IntakeConstants.uppyDownyMinPosition;
import static frc.robot.Constants.ShooterConstants.HoodConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants.AutoAlignConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.QuadranglesUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RobotCommands {
  private final Climber climber;
  private final Drive drive;
  private final Hopper hopper;
  private final Intake intake;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;

  private ShooterAimModel shooterAimModel;

  // ==================== CLIMBER ====================
  private LoggedNetworkNumber climberUpPos =
      new LoggedNetworkNumber("Tunable/ClimberUpPos", climberUpPosition);
  public LoggedNetworkNumber climberDownPos =
      new LoggedNetworkNumber("Tunable/ClimberDownPos", climberDownPosition);
  // How far DOWN the climber is for "mid" position
  public LoggedNetworkNumber climberMidFactor =
      new LoggedNetworkNumber("Tunable/ClimberMidFactor", 0.8);

  // ==================== HOPPER ====================
  private LoggedNetworkNumber spindexerSpeed = new LoggedNetworkNumber("Tunable/SpindexerRPM", 80);
  private LoggedNetworkNumber kickerSpeedFactor =
      new LoggedNetworkNumber(
          "Tunable/KickerSpeedFactor", 1.0); // Number to multiply flywheel speed by
  private boolean spindexerInverted = false;

  // ==================== INTAKE ====================
  private LoggedNetworkNumber intakeSpeed = new LoggedNetworkNumber("Tunable/IntakeRPM", 2000);

  // ==================== FLYWHEEL ====================
  private LoggedNetworkNumber flywheelThreshold =
      new LoggedNetworkNumber("Tunable/FlywheelThreshold", 0.99);
  public LoggedNetworkNumber flywheelSpeed = new LoggedNetworkNumber("Tunable/FlywheelRPM", 3000);

  // ==================== HOOD ====================
  private LoggedNetworkNumber hoodAngle =
      new LoggedNetworkNumber("Tunable/HoodAngle", 0.0 + hoodMinAngle.getDegrees());
  private LoggedNetworkNumber hoodIncrement =
      new LoggedNetworkNumber("Tunable/HoodIncrementDeg", 2.0);

  // ==================== TURRET ====================
  private LoggedNetworkNumber turretManualSpeed =
      new LoggedNetworkNumber("Tunable/TurretManualSpeed", 2.0);

  // ==================== COMMANDS ====================
  public final Command joystickDriveCommand;

  public RobotCommands(
      Climber climber,
      Drive drive,
      Hopper hopper,
      Intake intake,
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      ShooterAimModel shooterAimModel) {
    this.climber = climber;
    this.drive = drive;
    this.hopper = hopper;
    this.intake = intake;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;

    this.shooterAimModel = shooterAimModel;

    joystickDriveCommand =
        DriveCommands.joystickDrive(
            drive,
            OI.DriveOI::joystickDriveX,
            OI.DriveOI::joystickDriveY,
            OI.DriveOI::joystickDriveOmega);
  }

  // #region ==================== WHOLE ROBOT ====================

  public Command rezeroMechanisms() {
    return parallel(rezeroClimber(), rezeroHood());
    // TODO: intake uppy downy
  }

  // #endregion

  // #region ==================== CLIMBER ====================
  public Command climberUp() {
    return sequence(
        runOnce(
            () -> {
              climber.setCurrentLimit(Amps.of(climberCurrentLimit));
            },
            climber),
        runOnce(
            () -> {
              climber.setPosition(climberUpPos.get());
            },
            climber),
        waitUntil(() -> climber.getPosition() < climberUpPos.get() + climberTolerance),
        stopClimber());
  }

  public Command climberUpInstant() {
    return sequence(
        runOnce(
            () -> {
              climber.setCurrentLimit(Amps.of(climberCurrentLimit));
            },
            climber),
        runOnce(
            () -> {
              climber.setPosition(climberUpPos.get());
            },
            climber));
  }

  public Command climberMid() {
    return sequence(
        runOnce(
            () -> {
              climber.setCurrentLimit(Amps.of(climberCurrentLimit));
            },
            climber),
        runOnce(
            () -> {
              climber.setPosition(climberDownPos.get() * climberMidFactor.get());
            },
            climber),
        either(
            waitUntil(
                () ->
                    climber.getPosition()
                        > climberDownPos.get() * climberMidFactor.get() - climberTolerance),
            waitUntil(
                () ->
                    climber.getPosition()
                        < climberDownPos.get() * climberMidFactor.get() + climberTolerance),
            () -> climber.getPosition() <= climberDownPos.get() * climberMidFactor.get()),
        stopClimber());
  }

  public Command climberMidWithCurrent() {
    return sequence(
            runOnce(
                () -> {
                  climber.setCurrentLimit(Amps.of(30));
                  climber.setOpenLoop(Volts.of(5));
                },
                climber),
            waitUntil(() -> climber.getFilteredCurrent().gte(Amps.of(25))),
            runOnce(
                () -> {
                  climber.setCurrentLimit(Amps.of(climberCurrentLimit));
                },
                climber),
            waitSeconds(0.5),
            climberMid())
        .finallyDo(
            () -> {
              climber.setOpenLoop(Volts.of(0));
              climber.setCurrentLimit(Amps.of(climberCurrentLimit));
            });
  }

  public Command climberDown() {
    return sequence(
        runOnce(
            () -> {
              climber.setCurrentLimit(Amps.of(climberCurrentLimit));
            },
            climber),
        runOnce(
            () -> {
              climber.setPosition(climberDownPos.get());
            },
            climber),
        waitUntil(() -> climber.getPosition() > climberDownPos.get() - climberTolerance),
        stopClimber());
  }

  public Command rezeroClimber() {
    return sequence(
            runOnce(
                () -> {
                  climber.setCurrentLimit(Amps.of(20));
                  climber.setOpenLoop(Volts.of(2));
                },
                climber),
            waitUntil(() -> climber.getFilteredCurrent().gte(Amps.of(19))),
            runOnce(
                () -> {
                  climber.setOpenLoop(Volts.of(0));
                  climber.setRelativeEncoderPosition(climberDownPosition);
                },
                climber),
            runOnce(
                () -> {
                  climber.setCurrentLimit(Amps.of(climberCurrentLimit));
                },
                climber))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .finallyDo(
            () -> {
              climber.setOpenLoop(Volts.of(0));
              climber.setCurrentLimit(Amps.of(climberCurrentLimit));
            });
  }

  public Command stopClimber() {
    return runOnce(
        () -> {
          climber.setOpenLoop(Volts.of(0));
        },
        climber);
  }

  public Command climberManualUp() {
    return sequence(
        runOnce(
            () -> {
              climber.setOpenLoop(Volts.of(-2));
            },
            climber),
        waitUntil(() -> climber.getPosition() <= climberUpPosition + 0.05),
        stopClimber());
  }

  public Command climberManualDown() {
    return sequence(
        runOnce(
            () -> {
              climber.setOpenLoop(Volts.of(2));
            },
            climber),
        waitUntil(() -> climber.getPosition() >= climberDownPosition - 0.05),
        stopClimber());
  }

  // #endregion

  // #region ==================== DRIVE ====================

  public Command stopDrive() {
    return runOnce(() -> drive.stop(), drive);
  }

  public Command creepForward() {
    return run(
        () -> {
          drive.runVelocity(
              new ChassisSpeeds(MetersPerSecond.of(0.75), MetersPerSecond.of(0), RPM.of(0)));
        },
        drive);
  }

  public Command creepBackward() {
    return run(
        () -> {
          drive.runVelocity(
              new ChassisSpeeds(MetersPerSecond.of(-0.25), MetersPerSecond.of(0), RPM.of(0)));
        },
        drive);
  }

  public Command sprintForward() {
    return run(
        () -> {
          drive.runVelocity(
              new ChassisSpeeds(MetersPerSecond.of(10.0), MetersPerSecond.of(0), RPM.of(0)));
        },
        drive);
  }

  public Command sprintBackward() {
    return run(
        () -> {
          drive.runVelocity(
              new ChassisSpeeds(MetersPerSecond.of(-10.0), MetersPerSecond.of(0), RPM.of(0)));
        },
        drive);
  }

  public Command autoAlignClimb() {
    return runOnce(
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
                    new AutoAlignCommand(AutoAlignConstants.climbPoseOutpost, drive),
                    creepBackward()));
          } else {
            drive.setDefaultCommand(
                sequence(
                    new AutoAlignCommand(AutoAlignConstants.climbSetupPoseDepot, drive),
                    new AutoAlignCommand(AutoAlignConstants.climbPoseDepot, drive),
                    creepBackward()));
          }
        },
        drive);
  }

  // #endregion

  // #region ==================== HOPPER ====================

  public Command runSpindexer() {
    return runOnce(
        () -> {
          hopper.setSpindexerVelocity(RPM.of((spindexerInverted ? 1 : 1) * spindexerSpeed.get()));
        },
        hopper);
  }

  public Command runSpindexerSlow() {
    return runOnce(
        () -> {
          hopper.setSpindexerVelocity(
              RPM.of((spindexerInverted ? 1 : 1) * spindexerSpeed.get() / 8.0));
        },
        hopper);
  }

  public Command runSpindexerReverse() {
    return runOnce(
        () -> {
          hopper.setSpindexerVelocity(RPM.of((spindexerInverted ? -1 : -1) * spindexerSpeed.get()));
        },
        hopper);
  }

  public Command stopSpindexer() {
    return runOnce(
        () -> {
          hopper.setSpindexerVelocity(RPM.of(0.0));
        },
        hopper);
  }

  public Command startKicker() {
    return runOnce(
        () -> {
          hopper.setKickerVelocity(RPM.of(flywheelSpeed.get() * kickerSpeedFactor.get()));
        },
        hopper);
  }

  public Command startKickerReverse() {
    return runOnce(
        () -> {
          hopper.setKickerVelocity(RPM.of(flywheelSpeed.get() * -kickerSpeedFactor.get()));
        },
        hopper);
  }

  public Command stopKicker() {
    return runOnce(
        () -> {
          hopper.setKickerVelocity(RPM.of(0.0));
        },
        hopper);
  }

  public Command jiggleRobot() {
    return repeatingSequence(sprintForward().withTimeout(0.5), sprintBackward().withTimeout(0.5));
  }

  public Command runSpindexerWithStallDetection(Supplier<AngularVelocity> velocity) {
    return run(() -> hopper.setSpindexerVelocity(velocity.get()), hopper)
        .until(() -> hopper.getSpindexerFilteredCurrent().gt(Amps.of(spindexerCurrentLimit - 2.0)))
        .andThen(
            runOnce(() -> hopper.setSpindexerVelocity(velocity.get().times(-1.0)), hopper)
                .andThen(Commands.waitSeconds(2.0)))
        .repeatedly();
  }

  public Command runSpindexerKickerWithStallDetection(Supplier<AngularVelocity> velocity) {
    return sequence(
        runSpindexer(),
        startKicker(),
        repeatingSequence(
            waitUntil(
                () ->
                    hopper.getSpindexerFilteredCurrent().gt(Amps.of(spindexerCurrentLimit - 2.0))),
            runSpindexerReverse(),
            stopKicker(),
            waitSeconds(0.1),
            startKicker(),
            waitUntil(
                () ->
                    hopper.getSpindexerFilteredCurrent().gt(Amps.of(spindexerCurrentLimit - 2.0))),
            runSpindexer(),
            stopKicker(),
            waitSeconds(0.1),
            startKicker()));
  }

  public Command unjamSpindexer() {
    return sequence(
        runOnce(
            () -> {
              hopper.setKickerVelocity(
                  RPM.of(flywheelSpeed.get() * kickerSpeedFactor.get()).times(-1.0));
            },
            hopper),
        runOnce(
            () -> {
              hopper.setSpindexerVelocity(
                  RPM.of((spindexerInverted ? -1 : -1) * spindexerSpeed.get()));
            },
            hopper));
  }

  // #endregion

  // #region ==================== INTAKE ====================

  public Command intake() {
    return parallel(
        runIntake(), runSpindexerWithStallDetection(() -> RPM.of(spindexerSpeed.get() / 8.0)));
  }

  public Command releaseIntake() {
    return sequence(stopIntake(), stopSpindexer());
  }

  public Command runIntake() {
    return runOnce(
        () -> {
          intake.setSpinnySpinnyVelocity(RPM.of(intakeSpeed.get()));
        },
        intake);
  }

  public Command runIntakeReverse() {
    return runOnce(
        () -> {
          intake.setSpinnySpinnyVelocity(RPM.of(-intakeSpeed.get()));
        },
        intake);
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          intake.setSpinnySpinnyVelocity(RPM.of(0.0));
        },
        intake);
  }

  public Command dropIntake() {
    return sequence(
        sprintForward().withTimeout(0.88), sprintBackward().withTimeout(0.75), stopDrive());
  }

  public Command rezeroIntakeUppyDowny() {
    return sequence(
        runOnce(
            () -> {
              intake.setUppyDownyCurrentLimit(Amps.of(20));
              intake.setUppyDownyOpenLoop(Volts.of(2));
            },
            intake),
        waitUntil(() -> intake.getUppyDownyFilteredCurrent().gte(Amps.of(19))),
        runOnce(
            () -> {
              intake.setUppyDownyOpenLoop(Volts.of(0));
              // TODO: min or max?
              intake.setUppyDownyRelativeEncoderPosition(uppyDownyMinPosition);
              intake.setUppyDownyCurrentLimit(Amps.of(uppyDownyCurrentLimit));
            },
            intake));
  }

  public Command jostleIntake() {
    // Start by moving up (-RPM) first so we move away from the bottom hard stop
    return sequence(
            runOnce(() -> intake.setUppyDownyVelocity(intake.getUppyDownyRaiseRPM()), intake),
            waitSeconds(0.15),
            repeatingSequence(
                run(() -> intake.setUppyDownyVelocity(intake.getUppyDownyLowerRPM()), intake)
                    .withTimeout(intake.getInstakeDownTime()),
                run(() -> intake.setUppyDownyVelocity(intake.getUppyDownyRaiseRPM()), intake)
                    .withTimeout(intake.getInstakeUpTime())))
        .finallyDo(() -> intake.setUppyDownyVelocity(RPM.of(0)));
  }

  public Command ceaseJostleIntake() {
    return runOnce(
        () -> {
          intake.setUppyDownyVelocity(RPM.of(0));
        },
        intake);
  }

  public Command raiseIntake() {
    return run(() -> intake.setUppyDownyVelocity(intake.getUppyDownyRaiseRPM()), intake)
        .finallyDo(() -> intake.setUppyDownyVelocity(RPM.of(0)));
  }

  public Command lowerIntake() {
    return run(() -> intake.setUppyDownyVelocity(intake.getUppyDownyLowerRPM()), intake)
        .finallyDo(() -> intake.setUppyDownyVelocity(RPM.of(0)));
  }

  // #endregion

  // #region ==================== SHOOTER ====================

  public Command shoot() {
    return sequence(
        startHood(),
        startFlywheel(),
        waitUntil(() -> flywheel.atVelocity(flywheelThreshold.get())),
        runOnce(
            () -> {
              spindexerInverted = !spindexerInverted;
            }),
        startKicker(),
        runIntake(),
        parallel(
            autoFlywheelCommand(),
            autoHoodCommand(),
            jostleIntake(),
            runSpindexerKickerWithStallDetection(() -> RPM.of(spindexerSpeed.get()))));
  }

  public Command shootWithoutIntakeJostle() {
    return sequence(
        startHood(),
        startFlywheel(),
        waitUntil(() -> flywheel.atVelocity(flywheelThreshold.get())),
        runOnce(
            () -> {
              spindexerInverted = !spindexerInverted;
            }),
        startKicker(),
        runIntake(),
        parallel(
            autoFlywheelCommand(),
            autoHoodCommand(),
            runSpindexerKickerWithStallDetection(() -> RPM.of(spindexerSpeed.get()))));
  }

  public Command spinDownFromShoot() {
    return sequence(
        stopSpindexer(),
        stopHood(),
        waitSeconds(0.25),
        stopKicker(),
        waitSeconds(0.25),
        stopIntake(),
        ceaseJostleIntake(),
        ceaseFlywheel());
  }

  public Command stopShootingNoDelay() {
    return sequence(stopSpindexer(), stopKicker(), stopIntake(), stopHood(), ceaseFlywheel());
  }

  public Command enableAutoShooterSettings() {
    return runOnce(
        () -> {
          flywheel.setDefaultCommand(autoFlywheelCommand());
          hood.setDefaultCommand(autoHoodCommand());
        },
        flywheel,
        hood);
  }

  public Command setManualShooterSettings(Rotation2d hoodAngle, AngularVelocity flywheelSpeed) {
    return sequence(runFlywheelManual(flywheelSpeed), setHoodManual(hoodAngle));
  }

  public Command setManualShooterSettingsWithTrim(
      Rotation2d hoodAngle, AngularVelocity flywheelSpeed) {
    return sequence(
        runFlywheelManual(() -> shooterAimModel.applyFlywheelTrim(flywheelSpeed)),
        setHoodManual(() -> shooterAimModel.applyHoodTrim(hoodAngle)));
  }

  public Command setCloseShot(boolean withTrim) {
    Rotation2d angle = Rotation2d.fromDegrees(25.6);
    AngularVelocity speed = RPM.of(2800);
    if (!withTrim) {
      return setManualShooterSettings(angle, speed);
    } else {
      return setManualShooterSettingsWithTrim(angle, speed);
    }
  }

  public Command setMediumShot(boolean withTrim) {
    Rotation2d angle = Rotation2d.fromDegrees(33);
    AngularVelocity speed = RPM.of(3100);
    if (!withTrim) {
      return setManualShooterSettings(angle, speed);
    } else {
      return setManualShooterSettingsWithTrim(angle, speed);
    }
  }

  public Command setFarShot(boolean withTrim) {
    Rotation2d angle = Rotation2d.fromDegrees(38);
    AngularVelocity speed = RPM.of(3675);
    if (!withTrim) {
      return setManualShooterSettings(angle, speed);
    } else {
      return setManualShooterSettingsWithTrim(angle, speed);
    }
  }

  public Command setNeutralZoneShot(boolean withTrim) {
    Rotation2d angle = Rotation2d.fromDegrees(45);
    AngularVelocity speed = RPM.of(4500);
    if (!withTrim) {
      return setManualShooterSettings(angle, speed);
    } else {
      return setManualShooterSettingsWithTrim(angle, speed);
    }
  }

  public Command setDashboardShot() {
    return sequence(
        runFlywheelManual(() -> RPM.of(flywheelSpeed.get())),
        setHoodManual(() -> Rotation2d.fromDegrees(hoodAngle.get())));
  }

  public void setShooterAimModel(ShooterAimModel shooterAimModel) {
    this.shooterAimModel = shooterAimModel;
  }

  // #endregion

  // #region ==================== FLYWHEEL ====================

  public Command autoFlywheelCommand() {
    return run(
        () -> {
          flywheel.setVelocity(shooterAimModel.getFlywheelSpeed());
        },
        flywheel);
  }

  public Command stopFlywheel() {
    return runOnce(
        () -> {
          flywheel.removeDefaultCommand();
          flywheel.setVelocity(RPM.of(0));
        },
        flywheel);
  }

  public Command startFlywheel() {
    return runOnce(
        () -> {
          flywheel.setShooting(true);
        },
        flywheel);
  }

  public Command ceaseFlywheel() {
    return runOnce(
        () -> {
          flywheel.setShooting(false);
        },
        flywheel);
  }

  public Command runFlywheelManual(AngularVelocity speed) {
    return runOnce(
        () -> {
          flywheel.removeDefaultCommand();
          flywheel.setVelocity(speed);
        },
        flywheel);
  }

  public Command runFlywheelManual(Supplier<AngularVelocity> speed) {
    return runOnce(
        () -> {
          flywheel.removeDefaultCommand();
          flywheel.setVelocity(speed.get());
        },
        flywheel);
  }

  // #endregion

  // #region ==================== HOOD ====================

  public Command autoHoodCommand() {
    return run(
        () -> {
          hood.setPosition(shooterAimModel.getHoodAngle());
        },
        hood);
  }

  public Command startHood() {
    return runOnce(
        () -> {
          hood.setShooting(true);
        },
        hood);
  }

  public Command stopHood() {
    return runOnce(
        () -> {
          hood.setShooting(false);
        },
        hood);
  }

  public Command rezeroHood() {
    return sequence(
            runOnce(
                () -> {
                  hood.setCurrentLimit(Amps.of(20));
                  hood.setOpenLoop(Volts.of(-1));
                },
                hood),
            waitUntil(() -> hood.getFilteredCurrent().gte(Amps.of(19))),
            runOnce(
                () -> {
                  hood.setOpenLoop(Volts.of(0));
                  hood.setRelativeEncoderPosition(hoodMinAngle);
                  hood.setCurrentLimit(Amps.of(hoodCurrentLimit));
                },
                hood))
        .withTimeout(hoodRezeroTimeoutSeconds)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command setHoodManual(Rotation2d angle) {
    return runOnce(
        () -> {
          hood.removeDefaultCommand();
          hood.setPosition(angle);
        },
        hood);
  }

  public Command setHoodManual(Supplier<Rotation2d> angle) {
    return runOnce(
        () -> {
          hood.removeDefaultCommand();
          hood.setPosition(angle.get());
        },
        hood);
  }

  public Command hoodManualUp() {
    return run(
        () -> {
          hood.removeDefaultCommand();
          hood.setPosition(
              hood.getHoodSetpoint().plus(Rotation2d.fromDegrees(hoodIncrement.get())));
        },
        hood);
  }

  public Command hoodManualDown() {
    return run(
        () -> {
          hood.removeDefaultCommand();
          hood.setPosition(
              hood.getHoodSetpoint().minus(Rotation2d.fromDegrees(hoodIncrement.get())));
        },
        hood);
  }

  // #endregion

  // #region ==================== TURRET ====================

  public Command autoTurretCommand() {
    return run(
        () -> {
          turret.setTurretArbFF(shooterAimModel.getTurretFF());
          turret.setPosition(shooterAimModel.getTurretAngleRot());
        },
        turret);
  }

  public Command turretManualCCW() {
    return sequence(
        run(
            () -> {
              turret.removeDefaultCommand();
              turret.setPosition(
                  turret.getTurretSetpointRot()
                      + Units.degreesToRotations(turretManualSpeed.get()));
            },
            turret));
  }

  public Command turretManualCW() {
    return sequence(
        run(
            () -> {
              turret.removeDefaultCommand();
              turret.setPosition(
                  turret.getTurretSetpointRot()
                      - Units.degreesToRotations(turretManualSpeed.get()));
            },
            turret));
  }

  public Command turretToPosition(double rotations) {
    return runOnce(
        () -> {
          turret.removeDefaultCommand();
          turret.setPosition(rotations);
        },
        turret);
  }

  public Command lockTurret() {
    return turretToPosition(Units.degreesToRotations(90.0));
  }

  public Command enableAutoTurret() {
    return runOnce(
            () -> {
              turret.setDefaultCommand(autoTurretCommand());
            },
            turret)
        .ignoringDisable(true);
  }

  public Command setTurretEncoderTo0() {
    return runOnce(
            () -> {
              turret.setRelativeEncoderPosition(0);
            },
            turret)
        .ignoringDisable(true);
  }

  public Command setTurretEncoderTo180() {
    return runOnce(
            () -> {
              turret.setRelativeEncoderPosition(0.5);
            },
            turret)
        .ignoringDisable(true);
  }

  // #endregion

}
