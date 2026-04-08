package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.HopperConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;
import static frc.robot.Constants.ShooterConstants.FlywheelConstants.*;
import static frc.robot.Constants.ShooterConstants.HoodConstants.*;
import static frc.robot.Constants.ShooterConstants.TurretConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
import frc.robot.util.choreo.ChoreoVars;

import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotCommands {
  private final Climber climber;
  private final Drive drive;
  private final Hopper hopper;
  private final Intake intake;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;

  private ShooterAimModel shooterAimModel;

  private boolean spindexerInverted = false;
  private int spindexerStallReversals = 0;
  private int spindexerStallsForward = 0;
  private int spindexerStallsReverse = 0;

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

  // #region WHOLE ROBOT

  // #endregion

  // #region CLIMBER
  public Command runClimberUp() {
    return sequence(
        runOnce(
            () -> {
              climber.setCurrentLimit(climberCurrentLimit);
            },
            climber),
        runOnce(
            () -> {
              climber.setPosition(climberUpPosition);
            },
            climber),
        waitUntil(() -> climber.getPosition() < climberUpPosition + climberTolerance),
        stopClimber());
  }

  public Command startClimberUp() {
    return sequence(
        runOnce(
            () -> {
              climber.setCurrentLimit(climberCurrentLimit);
            },
            climber),
        runOnce(
            () -> {
              climber.setPosition(climberUpPosition);
            },
            climber));
  }

  public Command runClimberMid() {
    return sequence(
        runOnce(
            () -> {
              climber.setCurrentLimit(climberCurrentLimit);
            },
            climber),
        runOnce(
            () -> {
              climber.setPosition(climberDownPosition * climbPositionFactor);
            },
            climber),
        either(
            waitUntil(
                () ->
                    climber.getPosition()
                        > climberDownPosition * climbPositionFactor - climberTolerance),
            waitUntil(
                () ->
                    climber.getPosition()
                        < climberDownPosition * climbPositionFactor + climberTolerance),
            () -> climber.getPosition() <= climberDownPosition * climbPositionFactor),
        stopClimber());
  }

  public Command runClimberMidWithCurrent() {
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
                  climber.setCurrentLimit(climberCurrentLimit);
                },
                climber),
            waitSeconds(0.5),
            runClimberMid())
        .finallyDo(
            () -> {
              climber.setOpenLoop(Volts.of(0));
              climber.setCurrentLimit(climberCurrentLimit);
            });
  }

  public Command runClimberDown() {
    return sequence(
        runOnce(
            () -> {
              climber.setCurrentLimit(climberCurrentLimit);
            },
            climber),
        runOnce(
            () -> {
              climber.setPosition(climberDownPosition);
            },
            climber),
        waitUntil(() -> climber.getPosition() > climberDownPosition - climberTolerance),
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
                  climber.setCurrentLimit(climberCurrentLimit);
                },
                climber))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .finallyDo(
            () -> {
              climber.setOpenLoop(Volts.of(0));
              climber.setCurrentLimit(climberCurrentLimit);
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

  // #region DRIVE

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

  public Command autoAlignToTower() {
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

  // #region HOPPER

  public Command startSpindexer() {
    return runOnce(
        () -> {
          hopper.setSpindexerVelocity(spindexerSpeed.times(spindexerInverted ? -1 : 1));
        },
        hopper);
  }

  public Command startSpindexerReverse() {
    return runOnce(
        () -> {
          hopper.setSpindexerVelocity(spindexerSpeed.times(spindexerInverted ? 1 : -1));
        },
        hopper);
  }

  public Command invertSpindexer() {
    return runOnce(
        () -> {
          if (spindexerInverted) {
            spindexerStallsReverse++;
          } else {
            spindexerStallsForward++;
          }
          spindexerInverted = !spindexerInverted;
          spindexerStallReversals++;
          Logger.recordOutput("Hopper/SpindexerStallReversals", spindexerStallReversals);
          Logger.recordOutput("Hopper/SpindexerStallsForward", spindexerStallsForward);
          Logger.recordOutput("Hopper/SpindexerStallsReverse", spindexerStallsReverse);
        });
  }

  public Command stopSpindexer() {
    return runOnce(
        () -> {
          hopper.setSpindexerVelocity(RPM.zero());
        },
        hopper);
  }

  public Command startKicker() {
    return runOnce(
        () -> {
          hopper.setKickerVelocity(kickerSpeed);
        },
        hopper);
  }

  public Command startKickerWithTrenchSafety() {
    return isUnderTrench() ? stopKicker() : startKicker();
  }

  public Command startKickerReverse() {
    return runOnce(
        () -> {
          hopper.setKickerVelocity(kickerSpeed.times(-1.0));
        },
        hopper);
  }

  public Command stopKicker() {
    return runOnce(
        () -> {
          hopper.setKickerVelocity(RPM.zero());
        },
        hopper);
  }

  public Command jiggleRobot() {
    return repeatingSequence(sprintForward().withTimeout(0.5), sprintBackward().withTimeout(0.5));
  }

  public Command runSpindexerWithStallDetection(Supplier<AngularVelocity> velocity) {
    return repeatingSequence(
            startSpindexer(),
            waitUntil(
                () ->
                    hopper
                        .getSpindexerFilteredCurrent()
                        .gt(spindexerCurrentLimit.minus(Amps.of(2.0)))),
            invertSpindexer())
        .finallyDo(
            () -> {
              spindexerInverted = false;
            });
  }

  public Command runSpindexerAndKicker(Supplier<AngularVelocity> velocity) {
    return sequence(
            startSpindexer(),
            startKickerWithTrenchSafety(),
            repeatingSequence(
                waitUntil(
                    () ->
                        false
                            || hopper
                                .getSpindexerFilteredCurrent()
                                .gt(spindexerCurrentLimit.minus(Amps.of(2.0)))
                            || !turret.withinShootingTolerance()),
                either(
                    sequence(
                        stopKicker(),
                        waitUntil(turret::withinShootingTolerance),
                        startKickerWithTrenchSafety()),
                    sequence(
                        invertSpindexer(),
                        startSpindexer(),
                        stopKicker(),
                        waitSeconds(0.1),
                        startKickerWithTrenchSafety()),
                    () -> !turret.withinShootingTolerance())))
        .finallyDo(
            () -> {
              spindexerInverted = false;
            });
  }

  // #endregion

  // #region INTAKE

  public Command intake() {
    return parallel(startIntake(), runSpindexerWithStallDetection(() -> spindexerIntakingSpeed));
  }

  public Command spinDownFromIntake() {
    return sequence(stopIntake(), stopSpindexer());
  }

  public Command startIntake() {
    return runOnce(
        () -> {
          intake.setSpinnySpinnyVelocity(intakeSpinnySpinnySpeed);
        },
        intake);
  }

  public Command startIntakeReverse() {
    return runOnce(
        () -> {
          intake.setSpinnySpinnyVelocity(intakeSpinnySpinnySpeed.times(-1.0));
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

  public Command dropIntakeWithDrive() {
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

  private Command upDownCommand(){
    return
        repeatingSequence(
            run(() -> intake.setUppyDownyVelocity(RPM.of(uppyDownyLowerRPM)), intake)
                .withTimeout(jostleIntakeDownTime),
            run(() -> intake.setUppyDownyVelocity(RPM.of(uppyDownyRaiseRPM)), intake)
                .withTimeout(jostleIntakeUpTime));
  }

  public Command runIntakeJostle() {
    // Start by moving up (-RPM) first so we move away from the bottom hard stop
    return sequence(
            runOnce(() -> intake.setUppyDownyVelocity(RPM.of(uppyDownyRaiseRPM)), intake),
            waitSeconds(0.15),
            deferredProxy(this::upDownCommand)
        .finallyDo(() -> intake.setUppyDownyVelocity(RPM.zero())));
  }

  public Command stopIntakeJostle() {
    return runOnce(
        () -> {
          intake.setUppyDownyVelocity(RPM.zero());
        },
        intake);
  }

  public Command intakeManualUp() {
    return run(() -> intake.setUppyDownyVelocity(RPM.of(uppyDownyRaiseRPM)), intake)
        .finallyDo(() -> intake.setUppyDownyVelocity(RPM.zero()));
  }

  public Command intakeManualDown() {
    return run(() -> intake.setUppyDownyVelocity(RPM.of(uppyDownyLowerRPM)), intake)
        .finallyDo(() -> intake.setUppyDownyVelocity(RPM.zero()));
  }

  // #endregion

  // #region SHOOTER

  public Command shoot() {
    return sequence(
        startHood(),
        startFlywheel(),
        // startIntake(),
        startSpindexer(),
        waitUntil(() -> flywheel.atVelocity(flywheelThresholdFactor)),
        waitUntil(turret::withinShootingTolerance),
        parallel(
            autoFlywheelCommand(),
            autoHoodCommandWithTrenchSafety(),
            runIntakeJostle(),
            runSpindexerAndKicker(() -> spindexerSpeed)));
  }

  public Command shootWithoutIntakeJostle() {
    return sequence(
        startHood(),
        startFlywheel(),
        startIntake(),
        startSpindexer(),
        waitUntil(() -> flywheel.atVelocity(flywheelThresholdFactor)),
        waitUntil(turret::withinShootingTolerance),
        parallel(
            autoFlywheelCommand(),
            autoHoodCommandWithTrenchSafety(),
            runSpindexerAndKicker(() -> spindexerSpeed)));
  }

  public Command spinDownFromShoot() {
    return sequence(
        stopSpindexer(),
        stopHood(),
        waitSeconds(0.25),
        stopKicker(),
        waitSeconds(0.25),
        stopIntake(),
        stopIntakeJostle(),
        stopFlywheel());
  }

  public Command stopShootNoDelay() {
    return sequence(stopSpindexer(), stopKicker(), stopIntake(), stopHood(), stopFlywheel());
  }

  public Command resetDistanceTrim() {
    return runOnce(
            () -> {
              if (shooterAimModel.isInAllianceZone()) {
                azDistanceTrim = azDistanceTrimDefault;
              } else {
                nzDistanceTrim = nzDistanceTrimDefault;
              }
            },
            shooterAimModel)
        .ignoringDisable(true);
  }

  public Command increaseDistanceTrim() {
    return runOnce(
            () -> {
              if (shooterAimModel.isInAllianceZone()) {
                azDistanceTrim = azDistanceTrim.plus(distanceTrimIncrement);
              } else {
                nzDistanceTrim = nzDistanceTrim.plus(distanceTrimIncrement);
              }
            },
            shooterAimModel)
        .ignoringDisable(true);
  }

  public Command decreaseDistanceTrim() {
    return runOnce(
            () -> {
              if (shooterAimModel.isInAllianceZone()) {
                azDistanceTrim = azDistanceTrim.minus(distanceTrimIncrement);
              } else {
                nzDistanceTrim = nzDistanceTrim.minus(distanceTrimIncrement);
              }
            },
            shooterAimModel)
        .ignoringDisable(true);
  }

  public Command resetXYTrim() {
    return runOnce(
            () -> {
              if (shooterAimModel.isInAllianceZone()) {
                azXTrim = Inches.of(0.0);
                azYTrim = Inches.of(0.0);
              } else {
                nzXTrim = Inches.of(0.0);
                nzYTrim = Inches.of(0.0);
              }
            },
            shooterAimModel)
        .ignoringDisable(true);
  }

  public Command changeXTrim(boolean increment) {
    return runOnce(
            () -> {
              if (increment) {
                if (shooterAimModel.isInAllianceZone()) {
                  azXTrim = azXTrim.plus(distanceTrimIncrement);
                } else {
                  nzXTrim = nzXTrim.plus(distanceTrimIncrement);
                }
              } else {
                if (shooterAimModel.isInAllianceZone()) {
                  azXTrim = azXTrim.minus(distanceTrimIncrement);
                } else {
                  nzXTrim = nzXTrim.minus(distanceTrimIncrement);
                }
              }
            },
            shooterAimModel)
        .ignoringDisable(true);
  }

  public Command changeYTrim(boolean increment) {
    return runOnce(
            () -> {
              if (increment) {
                if (shooterAimModel.isInAllianceZone()) {
                  azYTrim = azYTrim.plus(distanceTrimIncrement);
                } else {
                  nzYTrim = nzYTrim.plus(distanceTrimIncrement);
                }
              } else {
                if (shooterAimModel.isInAllianceZone()) {
                  azYTrim = azYTrim.minus(distanceTrimIncrement);
                } else {
                  nzYTrim = nzYTrim.minus(distanceTrimIncrement);
                }
              }
            },
            shooterAimModel)
        .ignoringDisable(true);
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
    return sequence(setFlywheelManual(flywheelSpeed), setHoodManual(hoodAngle));
  }

  public Command setManualShooterSettingsWithTrim(
      Rotation2d hoodAngle, AngularVelocity flywheelSpeed) {
    return sequence(
        setFlywheelManual(() -> shooterAimModel.applyFlywheelTrim(flywheelSpeed)),
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
        setFlywheelManual(() -> flywheelManualSpeed), setHoodManual(() -> hoodManualAngle));
  }

  public void setShooterAimModel(ShooterAimModel shooterAimModel) {
    this.shooterAimModel = shooterAimModel;
  }

  // #endregion

  // #region FLYWHEEL

  public Command autoFlywheelCommand() {
    return run(
        () -> {
          flywheel.setVelocity(shooterAimModel.getFlywheelSpeed());
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

  public Command stopFlywheel() {
    return runOnce(
        () -> {
          flywheel.setShooting(false);
        },
        flywheel);
  }

  public Command setFlywheelManual(AngularVelocity speed) {
    return runOnce(
        () -> {
          flywheel.removeDefaultCommand();
          flywheel.setVelocity(speed);
        },
        flywheel);
  }

  public Command setFlywheelManual(Supplier<AngularVelocity> speed) {
    return runOnce(
        () -> {
          flywheel.removeDefaultCommand();
          flywheel.setVelocity(speed.get());
        },
        flywheel);
  }

  public Command flywheelManualStop() {
    return runOnce(
        () -> {
          flywheel.removeDefaultCommand();
          flywheel.setVelocity(RPM.of(0));
        },
        flywheel);
  }

  // #endregion

  // #region HOOD

  public Command autoHoodCommand() {
    return run(
        () -> {
          hood.setPosition(shooterAimModel.getHoodAngle());
        },
        hood);
  }

  public Command autoHoodCommandWithTrenchSafety() {
    return run(
        () -> {
          if (isUnderTrench()) {
            hood.setShooting(false);
          } else {
            hood.setShooting(true);
            hood.setPosition(shooterAimModel.getHoodAngle());
          }
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

  public Command runHoodManualUp() {
    return run(
        () -> {
          hood.removeDefaultCommand();
          hood.setPosition(hood.getHoodSetpoint().plus(hoodManualIncrement));
        },
        hood);
  }

  public Command runHoodManualDown() {
    return run(
        () -> {
          hood.removeDefaultCommand();
          hood.setPosition(hood.getHoodSetpoint().minus(hoodManualIncrement));
        },
        hood);
  }

  // #endregion

  // #region TURRET

  public Command autoTurretCommand() {
    return run(
        () -> {
          turret.setTurretArbFF(shooterAimModel.getTurretFF());
          turret.setPosition(shooterAimModel.getTurretAngleRot());
        },
        turret);
  }

  public Command resetTurretTrim() {
    return runOnce(
            () -> {
              turretTrimRot = turretTrimDefaultRot;
            },
            shooterAimModel)
        .ignoringDisable(true);
  }

  public Command changeTurretTrim(boolean increment) {
    return runOnce(
            () -> {
              if (increment) {
                turretTrimRot += turretTrimIncrementRot;
              } else {
                turretTrimRot -= turretTrimIncrementRot;
              }
            },
            shooterAimModel)
        .ignoringDisable(true);
  }

  public Command runTurretManualCCW() {
    return sequence(
        run(
            () -> {
              turret.removeDefaultCommand();
              turret.setPosition(turret.getTurretSetpointRot() + turretManualIncrementRot);
            },
            turret));
  }

  public Command runTurretManualCW() {
    return sequence(
        run(
            () -> {
              turret.removeDefaultCommand();
              turret.setPosition(turret.getTurretSetpointRot() - turretManualIncrementRot);
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

  public Command rezeroTurret() {
    return runOnce(
            () -> {
              turret.setRelativeEncoderPosition(turretRezeroLocationRot);
            },
            turret)
        .ignoringDisable(true);
  }

  // #endregion

  // #region UTIL

  public boolean isUnderTrench() {
    double robotBlueX =
        QuadranglesUtil.toAllianceTranslation(drive.getPose().getTranslation()).getX();
    double hubX = ChoreoVars.Poses.Hub.getX();
    double speedX = drive.getChassisSpeeds().vxMetersPerSecond;
    double dynamicBuffer = trenchSafetyBufferMeters + speedX * trenchSafetyLookaheadSeconds;
    return Math.abs(robotBlueX - hubX) < dynamicBuffer;
  }
  // #endregion

}
