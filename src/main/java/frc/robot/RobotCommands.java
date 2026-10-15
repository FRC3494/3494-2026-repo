package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;
import static frc.robot.Constants.HopperConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;
import static frc.robot.Constants.ShooterConstants.FlywheelConstants.*;
import static frc.robot.Constants.ShooterConstants.HoodConstants.*;
import static frc.robot.Constants.ShooterConstants.TurretConstants.*;
import static frc.robot.util.QuadranglesUtil.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.SpinnySpinny;
import frc.robot.subsystems.intake.UppyDowny;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotCommands {
  private final Climber climber;
  private final Drive drive;
  private final Hopper hopper;
  private final SpinnySpinny spinnySpinny;
  private final UppyDowny uppyDowny;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;

  private ShooterAimModel shooterAimModel;

  private boolean spindexerInverted = spindexerDefaultDirection;
  private int spindexerStallReversals = 0;
  private int spindexerStallsForward = 0;
  private int spindexerStallsReverse = 0;
  private double spindexerUnjamStartPosition = 0.0;

  public final Command joystickDriveCommand;

  public RobotCommands(
      Climber climber,
      Drive drive,
      Hopper hopper,
      SpinnySpinny spinnySpinny,
      UppyDowny uppyDowny,
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      ShooterAimModel shooterAimModel) {
    this.climber = climber;
    this.drive = drive;
    this.hopper = hopper;
    this.spinnySpinny = spinnySpinny;
    this.uppyDowny = uppyDowny;
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
            waitUntil(() -> climber.getPosition() > climberUpPosition - climberTolerance),
            stopClimber())
        .withName("RunClimberUp");
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
                climber))
        .withName("StartClimberUp");
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
                  shooterAimModel.setTurretTrim(
                      shooterAimModel.getTurretTrimRot()
                          + (closerToWithFlip(
                                  climbPoseDepot_BLUE.getTranslation(),
                                  climbPoseOutpost_BLUE.getTranslation(),
                                  drive.getPose().getTranslation())
                              ? Units.degreesToRotations(-10.0)
                              : Units.degreesToRotations(10)));
                },
                shooterAimModel),
            runOnce(
                () -> {
                  climber.setPosition(climberClimbPosition);
                },
                climber),
            either(
                waitUntil(() -> climber.getPosition() > climberClimbPosition - climberTolerance),
                waitUntil(() -> climber.getPosition() < climberClimbPosition + climberTolerance),
                () -> climber.getPosition() <= climberClimbPosition),
            stopClimber())
        .withName("RunClimberMid");
  }

  public Command runClimberMidWithCurrent() {
    return sequence(
            runOnce(
                () -> {
                  climber.setCurrentLimit(Amps.of(30));
                  climber.setVoltage(Volts.of(-5));
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
              climber.setVoltage(Volts.zero());
              climber.setCurrentLimit(climberCurrentLimit);
            })
        .withName("RunClimberMidWithCurrent");
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
            waitUntil(() -> climber.getPosition() < climberDownPosition + climberTolerance),
            stopClimber())
        .withName("RunClimberDown");
  }

  public Command rezeroClimber() {
    return sequence(
            runOnce(
                () -> {
                  climber.setCurrentLimit(Amps.of(20));
                  climber.setVoltage(Volts.of(-4));
                },
                climber),
            waitUntil(() -> climber.getFilteredCurrent().gte(Amps.of(19))),
            runOnce(
                () -> {
                  climber.setVoltage(Volts.zero());
                  climber.setRelativeEncoderPosition(climberDownPosition);
                },
                climber),
            runOnce(
                () -> {
                  climber.setCurrentLimit(climberCurrentLimit);
                },
                climber),
            print("Climber Rezero Done ================================================="))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .finallyDo(
            () -> {
              climber.setVoltage(Volts.zero());
              climber.setCurrentLimit(climberCurrentLimit);
            })
        .withName("RezeroClimber");
  }

  public Command stopClimber() {
    return runOnce(
            () -> {
              climber.setVoltage(Volts.zero());
            },
            climber)
        .withName("StopClimber");
  }

  public Command climberManualUp() {
    return sequence(
            runOnce(
                () -> {
                  climber.setVoltage(Volts.of(2));
                },
                climber),
            waitUntil(() -> climber.getPosition() >= climberUpPosition - climberTolerance),
            stopClimber())
        .finallyDo(
            () -> {
              climber.setVoltage(Volts.zero());
            })
        .withName("ClimberManualUp");
  }

  public Command climberManualDown() {
    return sequence(
            runOnce(
                () -> {
                  climber.setVoltage(Volts.of(-2));
                },
                climber),
            waitUntil(() -> climber.getPosition() <= climberDownPosition + climberTolerance),
            stopClimber())
        .finallyDo(
            () -> {
              climber.setVoltage(Volts.zero());
            })
        .withName("ClimberManualDown");
  }

  // #endregion

  // #region DRIVE

  public Command stopDrive() {
    return runOnce(() -> drive.stop(), drive).withName("StopDrive");
  }

  public Command creepForward() {
    return run(
            () -> {
              drive.runVelocity(
                  new ChassisSpeeds(
                      MetersPerSecond.of(0.75), MetersPerSecond.zero(), RadiansPerSecond.zero()));
            },
            drive)
        .withName("CreepForward");
  }

  public Command creepBackward() {
    return run(
            () -> {
              drive.runVelocity(
                  new ChassisSpeeds(
                      MetersPerSecond.of(-0.25), MetersPerSecond.zero(), RadiansPerSecond.zero()));
            },
            drive)
        .withName("CreepBackward");
  }

  public Command sprintForward() {
    return run(
            () -> {
              drive.runVelocity(
                  new ChassisSpeeds(
                      MetersPerSecond.of(10.0), MetersPerSecond.zero(), RadiansPerSecond.zero()));
            },
            drive)
        .withName("SprintForward");
  }

  public Command sprintBackward() {
    return run(
            () -> {
              drive.runVelocity(
                  new ChassisSpeeds(
                      MetersPerSecond.of(-10.0), MetersPerSecond.zero(), RadiansPerSecond.zero()));
            },
            drive)
        .withName("SprintBackward");
  }

  public Command spinRobot() {
    return run(() -> {
          drive.runVelocity(
              new ChassisSpeeds(
                  MetersPerSecond.zero(), MetersPerSecond.zero(), DegreesPerSecond.of(500)));
        })
        .withName("SpinRobot");
  }

  // #endregion

  // #region HOPPER

  public Command startSpindexer() {
    return runOnce(
            () -> {
              hopper.setSpindexerVelocity(spindexerSpeed.times(spindexerInverted ? -1 : 1));
            },
            hopper)
        .withName("StartSpindexer");
  }

  public Command startSpindexerWithSpeed(Supplier<AngularVelocity> speed) {
    return runOnce(
            () -> {
              hopper.setSpindexerVelocity(speed.get().times(spindexerInverted ? -1 : 1));
            },
            hopper)
        .withName("StartSpindexerWithSpeed");
  }

  public Command startSpindexerReverse() {
    return runOnce(
            () -> {
              hopper.setSpindexerVelocity(spindexerSpeed.times(spindexerInverted ? 1 : -1));
            },
            hopper)
        .withName("StartSpindexerReverse");
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
            })
        .withName("InvertSpindexer");
  }

  public Command stopSpindexer() {
    return runOnce(
            () -> {
              hopper.setSpindexerVelocity(RPM.zero());
            },
            hopper)
        .withName("StopSpindexer");
  }

  public Command startKicker() {
    return runOnce(
            () -> {
              hopper.setKickerVelocity(kickerSpeed);
            },
            hopper)
        .withName("StartKicker");
  }

  public Command startKickerWithTrenchSafety() {
    return either(stopKicker(), startKicker(), () -> drive.isUnderTrench())
        .withName("StartKickerWithTrenchSafety");
  }

  public Command startKickerReverse() {
    return runOnce(
            () -> {
              hopper.setKickerVelocity(kickerSpeed.times(-1.0));
            },
            hopper)
        .withName("StartKickerReverse");
  }

  public Command stopKicker() {
    return runOnce(
            () -> {
              hopper.setKickerVelocity(RPM.zero());
            },
            hopper)
        .withName("StopKicker");
  }

  public Command jiggleRobot() {
    return repeatingSequence(sprintForward().withTimeout(0.5), sprintBackward().withTimeout(0.5))
        .withName("JiggleRobot");
  }

  public Command unjamSpindexer(Supplier<AngularVelocity> velocity) {
    return sequence(
            runOnce(
                () -> {
                  spindexerStallReversals++;
                  Logger.recordOutput("Hopper/SpindexerStallUnjam", spindexerStallReversals);
                  spindexerUnjamStartPosition = hopper.getSpindexerPosition();
                }),
            // Briefly reverse
            runOnce(
                () -> hopper.setSpindexerVelocity(velocity.get().times(spindexerInverted ? 1 : -1)),
                hopper),
            // Wait until we've reversed by the unjam distance
            waitUntil(
                () ->
                    Math.abs(hopper.getSpindexerPosition() - spindexerUnjamStartPosition)
                        >= spindexerUnjamMotorRotations * spindexerGearRatio))
        .withName("UnjamSpindexer");
  }

  public Command runSpindexerWithStallDetection(Supplier<AngularVelocity> velocity) {
    return repeatingSequence(
            startSpindexerWithSpeed(velocity),
            waitUntil(
                () ->
                    spindexerUnjamEnabled
                        && hopper.getSpindexerFilteredCurrent().gt(spindexerCurrentThreshold)),
            unjamSpindexer(velocity))
        .withName("RunSpindexerWithStallDetection");
  }

  public Command runSpindexerAndKicker() {
    return sequence(
            startSpindexer(),
            startKickerWithTrenchSafety(),
            repeatingSequence(
                waitUntil(
                    () ->
                        false
                            || (spindexerUnjamEnabled
                                && hopper
                                    .getSpindexerFilteredCurrent()
                                    .gt(spindexerCurrentThreshold))
                            || !turret.withinShootingTolerance()
                            || hood.isUnderTrench(drive.getPose(), drive.getChassisSpeeds())),
                either(
                    sequence(
                        stopKicker(),
                        waitUntil(
                            () ->
                                turret.withinShootingTolerance()
                                    && !hood.isUnderTrench(
                                        drive.getPose(), drive.getChassisSpeeds())),
                        startKickerWithTrenchSafety()),
                    sequence(
                        unjamSpindexer(() -> spindexerSpeed),
                        startSpindexer(),
                        stopKicker(),
                        waitSeconds(0.1),
                        startKickerWithTrenchSafety()),
                    () ->
                        !turret.withinShootingTolerance()
                            || hood.isUnderTrench(drive.getPose(), drive.getChassisSpeeds()))))
        .finallyDo(
            () -> {
              resetSpindexerInversion();
            })
        .withName("RunSpindexerAndKicker");
  }

  private void resetSpindexerInversion() {
    spindexerInverted = spindexerDefaultDirection;
  }

  public Command dumpFuelNoSafety() {
    return parallel(
            spinnySpinny.runState(SpinnySpinny.State.OUTTAKE),
            sequence(
                setManualShooterSettings(Rotation2d.fromDegrees(45), RPM.of(2800)),
                turretToPosition(Units.degreesToRotations(0)),
                startHood(),
                startFlywheel(),
                waitUntil(turret::withinShootingTolerance),
                startKicker(),
                waitUntil(() -> flywheel.atVelocity(flywheelThresholdFactor)),
                runSpindexerWithStallDetection(() -> spindexerSpeed)))
        .finallyDo(
            () -> {
              turret.setDefaultCommand(autoTurretCommand());
            })
        .withName("DumpFuelNoSafety");
  }

  // #endregion

  // #region INTAKE

  public Command runIntakeWithUnjam() {
    return repeatingSequence(
            spinnySpinny
                .runState(SpinnySpinny.State.INTAKE)
                .until(() -> spinnySpinny.getFilteredCurrent().gt(spinnySpinnyCurrentThreshold)),
            spinnySpinny.runState(SpinnySpinny.State.OUTTAKE).withTimeout(Seconds.of(0.1)))
        .withName("RunIntakeWithUnjam");
  }

  public Command dropIntakeWithDrive() {
    return sequence(
            sprintForward().withTimeout(0.88), sprintBackward().withTimeout(0.75), stopDrive())
        .withName("DropIntakeWithDrive");
  }

  private Rotation2d robotRotationCache;

  public Command dropIntakeWithSpin() {
    return sequence(
            runOnce(
                () -> {
                  robotRotationCache = drive.getRotation().plus(Rotation2d.fromDegrees(90));
                }),
            spinRobot()
                .until(() -> drive.getRotation().getDegrees() < robotRotationCache.getDegrees()),
            spinRobot()
                .until(() -> drive.getRotation().getDegrees() >= robotRotationCache.getDegrees()),
            stopDrive())
        .withName("DropIntakeWithSpin");
  }

  private Command upDownCommand() {
    return repeatingSequence(
            runOnce(() -> uppyDowny.setOpenLoop(jostleIntakeVoltage.plus(Volts.of(uppyDownyKg)))),
            waitTime(jostleIntakeDownTime),
            runOnce(
                () ->
                    uppyDowny.setOpenLoop(
                        jostleIntakeVoltage.unaryMinus().minus(Volts.of(uppyDownyKg)))),
            waitTime(jostleIntakeUpTime))
        .withName("IntakeUpDown");
  }

  public Command runIntakeJostle() {
    return defer(this::upDownCommand, this.upDownCommand().getRequirements())
        .finallyDo(this::stopIntakeUppyDowny)
        .withName("RunIntakeJostle");
  }

  public Command runIntakeJostleWithTrenchSafety() {
    // Start by moving up (-RPM) first so we move away from the bottom hard stop
    return repeatingSequence(
            waitUntil(() -> !hood.isUnderTrench(drive.getPose(), drive.getChassisSpeeds())),
            defer(this::upDownCommand, this.upDownCommand().getRequirements())
                .until(() -> hood.isUnderTrench(drive.getPose(), drive.getChassisSpeeds())),
            runIntakeDown())
        .finallyDo(this::stopIntakeUppyDowny)
        .withName("RunIntakeJostleWithTrenchSafety");
  }

  public Command runIntakeToPosition(double position) {
    return sequence(
            runOnce(
                () -> {
                  uppyDowny.setPosition(position);
                },
                uppyDowny),
            waitUntil(() -> uppyDowny.isAtSetpoint()))
        .withName("RunIntakeToPosition");
  }

  public Command runIntakeToPosition(DoubleSupplier position) {
    return runIntakeToPosition(position.getAsDouble());
  }

  public Command runIntakeUp() {
    return runIntakeToPosition(() -> uppyDownyUpPosition).withName("RunIntakeUp");
  }

  public Command runIntakeDown() {
    return runIntakeToPosition(() -> uppyDownyDownPosition).withName("RunIntakeDown");
  }

  private void stopIntakeUppyDowny() {
    uppyDowny.setOpenLoop(Volts.zero());
  }

  public Command intakeManual(boolean upwards) {
    return run(
            () -> {
              uppyDowny.setOpenLoop(
                  upwards ? uppyDownyManualVoltage : uppyDownyManualVoltage.unaryMinus());
            },
            uppyDowny)
        .finallyDo(this::stopIntakeUppyDowny)
        .withName(upwards ? "IntakeManualUp" : "IntakeManualDown");
  }

  public Command rezeroIntake() {
    return sequence(
            runOnce(
                () -> {
                  uppyDowny.setCurrentLimit(Amps.of(20));
                  uppyDowny.setOpenLoop(Volts.of(1));
                },
                uppyDowny),
            waitUntil(() -> uppyDowny.getFilteredCurrent().gte(Amps.of(19))),
            runOnce(
                () -> {
                  uppyDowny.setOpenLoop(Volts.zero());
                  uppyDowny.setRelativeEncoderPosition(uppyDownyUpPosition);
                },
                uppyDowny),
            runOnce(
                () -> {
                  uppyDowny.setCurrentLimit(uppyDownyCurrentLimit);
                },
                uppyDowny),
            print("Intake Rezero Done ================================================="))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .finallyDo(
            () -> {
              uppyDowny.setOpenLoop(Volts.zero());
              uppyDowny.setCurrentLimit(uppyDownyCurrentLimit);
            })
        .withName("RezeroIntake");
  }

  // #endregion

  // #region SHOOTER

  public Command shoot() {
    return getShootCommand(
        "Shoot",
        autoFlywheelCommand(),
        runAutoHood(),
        spinnySpinny.runState(SpinnySpinny.State.INTAKE_WHILE_SHOOTING),
        runIntakeJostleWithTrenchSafety(),
        runSpindexerAndKicker());
  }

  public Command shootWithManualSettings() {
    return getShootCommand(
        "ShootWithManualSettings",
        runManualHoodWithTrenchSafety(),
        spinnySpinny.runState(SpinnySpinny.State.INTAKE_WHILE_SHOOTING),
        runIntakeJostleWithTrenchSafety(),
        runSpindexerAndKicker());
  }

  public Command shootWoIntakeJostle() {
    return getShootCommand(
        "ShootWoIntakeJostle",
        autoFlywheelCommand(),
        runAutoHood(),
        spinnySpinny.runState(SpinnySpinny.State.INTAKE_WHILE_SHOOTING),
        runSpindexerAndKicker());
  }

  public Command shootWithOuttakeWoJostle() {
    return getShootCommand(
        "ShootWithOuttakeWoJostle",
        autoFlywheelCommand(),
        runAutoHood(),
        spinnySpinny.runState(SpinnySpinny.State.OUTTAKE_WHILE_SHOOTING),
        runSpindexerAndKicker());
  }

  private Command getShootCommand(String name, Command... parallelCommands) {
    return sequence(
            parallel(
                defer(
                    () ->
                        spinnySpinny
                            .runState(SpinnySpinny.State.OUTTAKE_WHILE_SHOOTING)
                            .withTimeout(intakeSpinnySpinnyShootingReverseTime),
                    Set.of(spinnySpinny)),
                sequence(
                    startHoodWithTrenchSafety(),
                    startFlywheel(),
                    waitUntil(turret::withinShootingTolerance),
                    startKickerWithTrenchSafety(),
                    waitUntil(() -> flywheel.atVelocity(flywheelThresholdFactor)))),
            parallel(parallelCommands))
        .onlyIf(() -> !hood.isUnderTrench(drive.getPose(), drive.getChassisSpeeds()))
        .withName(name);
  }

  public Command spinDownFromShoot() {
    return sequence(
            stopSpindexer(),
            stopHood(),
            waitSeconds(0.25),
            stopKicker(),
            waitSeconds(0.25),
            stopFlywheel(),
            runIntakeDown())
        .withName("SpinDownFromShoot");
  }

  public Command stopShootNoDelay() {
    return sequence(
            stopSpindexer(),
            stopKicker(),
            stopHood(),
            stopFlywheel(),
            runOnce(this::stopIntakeUppyDowny, uppyDowny))
        .withName("StopShootNoDelay");
  }

  public Command manualShootRelease() {
    return sequence(spinDownFromShoot(), enableAutoShooterSettings())
        .withName("ManualShootRelease");
  }

  public Command resetDistanceTrim() {
    return runOnce(
            () -> {
              if (shooterAimModel.isInAllianceZone()) {
                azDistanceTrim = azDistanceTrimDefault;
                Logger.recordOutput("AimShooterMathLinear/AZDistanceTrim", azDistanceTrimDefault);

              } else {
                nzDistanceTrim = nzDistanceTrimDefault;
                Logger.recordOutput("AimShooterMathLinear/NZDistanceTrim", nzDistanceTrimDefault);
              }
            },
            shooterAimModel)
        .ignoringDisable(true)
        .withName("ResetDistanceTrim");
  }

  public Command changeDistanceTrim(boolean increment) {
    return run(
            () -> {
              if (shooterAimModel.isInAllianceZone()) {
                Distance trim =
                    increment
                        ? azDistanceTrim.plus(distanceTrimIncrement)
                        : azDistanceTrim.minus(distanceTrimIncrement);
                azDistanceTrim = trim;
                Logger.recordOutput("AimShooterMathLinear/AZDistanceTrim", trim);

              } else {
                Distance trim =
                    increment
                        ? nzDistanceTrim.plus(distanceTrimIncrement)
                        : nzDistanceTrim.minus(distanceTrimIncrement);
                nzDistanceTrim = trim;
                Logger.recordOutput("AimShooterMathLinear/NZDistanceTrim", trim);
              }
            },
            shooterAimModel)
        .ignoringDisable(true)
        .withName(increment ? "IncreaseDistanceTrim" : "DecreaseDistanceTrim");
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
        .ignoringDisable(true)
        .withName("ResetXYTrim");
  }

  public Command changeXTrim(boolean increment) {
    return runOnce(
            () -> {
              if (shooterAimModel.isInAllianceZone()) {
                azXTrim =
                    increment ? azXTrim.plus(xyTrimIncrement) : azXTrim.minus(xyTrimIncrement);
              } else {
                nzXTrim =
                    increment ? nzXTrim.plus(xyTrimIncrement) : nzXTrim.minus(xyTrimIncrement);
              }
            },
            shooterAimModel)
        .ignoringDisable(true)
        .withName(increment ? "IncreaseXTrim" : "DecreaseXTrim");
  }

  public Command changeYTrim(boolean increment) {
    return runOnce(
            () -> {
              if (shooterAimModel.isInAllianceZone()) {
                azYTrim =
                    increment ? azYTrim.plus(xyTrimIncrement) : azYTrim.minus(xyTrimIncrement);
              } else {
                nzYTrim =
                    increment ? nzYTrim.plus(xyTrimIncrement) : nzYTrim.minus(xyTrimIncrement);
              }
            },
            shooterAimModel)
        .ignoringDisable(true)
        .withName(increment ? "IncreaseYTrim" : "DecreaseYTrim");
  }

  public Command enableAutoShooterSettings() {
    return runOnce(
            () -> {
              flywheel.setDefaultCommand(autoFlywheelCommand());
              hood.setDefaultCommand(autoHoodCommand());
            },
            flywheel,
            hood)
        .withName("EnableAutoShooterSettings");
  }

  public Command setManualShooterSettings(Rotation2d hoodAngle, AngularVelocity flywheelSpeed) {
    return sequence(setFlywheelManual(flywheelSpeed), setHoodManual(hoodAngle))
        .withName("SetManualShooterSettings");
  }

  public Command setManualShooterSettingsWithTrim(
      Rotation2d hoodAngle, AngularVelocity flywheelSpeed) {
    return sequence(
            setFlywheelManual(() -> shooterAimModel.applyFlywheelTrim(flywheelSpeed)),
            setHoodManual(() -> shooterAimModel.applyHoodTrim(hoodAngle)))
        .withName("SetManualShooterSettingsWithTrim");
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
    Rotation2d angle = Rotation2d.fromDegrees(45);
    AngularVelocity speed = RPM.of(3500);
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
            setFlywheelManual(() -> flywheelManualSpeed), setHoodManual(() -> hoodManualAngle))
        .withName("SetDashboardShot");
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
            flywheel)
        .ignoringDisable(true)
        .withName("AutoFlywheelCommand");
  }

  public Command startFlywheel() {
    return runOnce(
            () -> {
              flywheel.setShooting(true);
            },
            flywheel)
        .withName("StartFlywheel");
  }

  public Command stopFlywheel() {
    return runOnce(
            () -> {
              flywheel.setShooting(false);
            },
            flywheel)
        .withName("StopFlywheel");
  }

  public Command setFlywheelManual(AngularVelocity speed) {
    return runOnce(
            () -> {
              flywheel.removeDefaultCommand();
              flywheel.setVelocity(speed);
            },
            flywheel)
        .withName("SetFlywheelManual");
  }

  public Command setFlywheelManual(Supplier<AngularVelocity> speed) {
    return runOnce(
            () -> {
              flywheel.removeDefaultCommand();
              flywheel.setVelocity(speed.get());
            },
            flywheel)
        .withName("SetFlywheelManual");
  }

  public Command flywheelManualStop() {
    return runOnce(
            () -> {
              flywheel.removeDefaultCommand();
              flywheel.setVelocity(RPM.zero());
            },
            flywheel)
        .withName("FlywheelManualStop");
  }

  // #endregion

  // #region HOOD

  public Command autoHoodCommand() {
    return run(
            () -> {
              if (hood.isUnderTrench(drive.getPose(), drive.getChassisSpeeds())) {
                hood.setShooting(false);
              }
              hood.setPosition(shooterAimModel.getHoodAngle());
            },
            hood)
        .ignoringDisable(true)
        .withName("AutoHoodCommand");
  }

  public Command runAutoHood() {
    return run(
            () -> {
              hood.setShooting(!hood.isUnderTrench(drive.getPose(), drive.getChassisSpeeds()));
              hood.setPosition(shooterAimModel.getHoodAngle());
            },
            hood)
        .withName("AutoHoodWithTrenchSafety");
  }

  public Command runManualHoodWithTrenchSafety() {
    return run(
            () -> {
              hood.setShooting(!hood.isUnderTrench(drive.getPose(), drive.getChassisSpeeds()));
            },
            hood)
        .withName("ManualHoodWithTrenchSafety");
  }

  public Command startHood() {
    return runOnce(
            () -> {
              hood.setShooting(true);
            },
            hood)
        .withName("StartHood");
  }

  public Command startHoodWithTrenchSafety() {
    return runOnce(
            () -> {
              hood.setShooting(!hood.isUnderTrench(drive.getPose(), drive.getChassisSpeeds()));
            },
            hood)
        .withName("StartHoodWithTrenchSafety");
  }

  public Command stopHood() {
    return runOnce(
            () -> {
              hood.setShooting(false);
            },
            hood)
        .withName("StopHood");
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
                  hood.setOpenLoop(Volts.zero());
                  hood.setRelativeEncoderPosition(hoodMinAngle);
                  hood.setCurrentLimit(Amps.of(hoodCurrentLimit));
                },
                hood),
            enableAutoShooterSettings(),
            print("Hood Rezero Done ======================================="))
        .withTimeout(hoodRezeroTimeoutSeconds)
        .finallyDo(
            () -> {
              hood.setOpenLoop(Volts.zero());
            })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("RezeroHood");
  }

  public Command setHoodManual(Rotation2d angle) {
    return runOnce(
            () -> {
              hood.removeDefaultCommand();
              hood.setPosition(angle);
            },
            hood)
        .withName("SetHoodManual");
  }

  public Command setHoodManual(Supplier<Rotation2d> angle) {
    return runOnce(
            () -> {
              hood.removeDefaultCommand();
              hood.setPosition(angle.get());
            },
            hood)
        .withName("SetHoodManual");
  }

  public Command runHoodManualUp() {
    return run(
            () -> {
              hood.removeDefaultCommand();
              hood.setPosition(hood.getHoodSetpoint().plus(hoodManualIncrement));
            },
            hood)
        .withName("RunHoodManualUp");
  }

  public Command runHoodManualDown() {
    return run(
            () -> {
              hood.removeDefaultCommand();
              hood.setPosition(hood.getHoodSetpoint().minus(hoodManualIncrement));
            },
            hood)
        .withName("RunHoodManualDown");
  }

  // #endregion

  // #region TURRET

  public Command autoTurretCommand() {
    return run(
            () -> {
              turret.setTurretArbFF(shooterAimModel.getTurretFF());
              turret.setPosition(shooterAimModel.getTurretAngleRot());
            },
            turret)
        .ignoringDisable(true)
        .withName("AutoTurretCommand");
  }

  public Command resetTurretTrim() {
    return runOnce(
            () -> {
              turretTrimRot = turretTrimDefaultRot;
              Logger.recordOutput("AimShooterMathLinear/TurretTrimRot", turretTrimDefaultRot);
              Logger.recordOutput(
                  "AimShooterMathLinear/TurretTrimDeg",
                  Units.rotationsToDegrees(turretTrimDefaultRot));
            },
            shooterAimModel)
        .ignoringDisable(true)
        .withName("ResetTurretTrim");
  }

  public Command changeTurretTrim(boolean increment) {
    return run(
            () -> {
              if (increment) {
                double trim = turretTrimRot + turretTrimIncrementRot;
                turretTrimRot = trim;
                Logger.recordOutput("AimShooterMathLinear/TurretTrimRot", trim);
                Logger.recordOutput(
                    "AimShooterMathLinear/TurretTrimDeg", Units.rotationsToDegrees(trim));
              } else {
                double trim = turretTrimRot - turretTrimIncrementRot;
                turretTrimRot = trim;
                Logger.recordOutput("AimShooterMathLinear/TurretTrimRot", trim);
                Logger.recordOutput(
                    "AimShooterMathLinear/TurretTrimDeg", Units.rotationsToDegrees(trim));
              }
            },
            shooterAimModel)
        .ignoringDisable(true)
        .withName(increment ? "IncreaseTurretTrim" : "DecreaseTurretTrim");
  }

  public Command runTurretManualCCW() {
    return sequence(
            run(
                () -> {
                  turret.removeDefaultCommand();
                  turret.setPosition(turret.getTurretSetpointRot() + turretManualIncrementRot);
                },
                turret))
        .withName("RunTurretManualCCW");
  }

  public Command runTurretManualCW() {
    return sequence(
            run(
                () -> {
                  turret.removeDefaultCommand();
                  turret.setPosition(turret.getTurretSetpointRot() - turretManualIncrementRot);
                },
                turret))
        .withName("RunTurretManualCW");
  }

  public Command turretToPosition(double rotations) {
    return runOnce(
            () -> {
              turret.removeDefaultCommand();
              turret.setPosition(rotations);
            },
            turret)
        .withName("TurretTo" + Units.rotationsToDegrees(rotations));
  }

  public Command lockTurret() {
    return turretToPosition(turretLockLocationRot).withName("LockTurret");
  }

  public Command enableAutoTurret() {
    return runOnce(
            () -> {
              turret.setDefaultCommand(autoTurretCommand());
            },
            turret)
        .ignoringDisable(true)
        .withName("EnableAutoTurret");
  }

  public Command rezeroTurret() {
    return runOnce(
            () -> {
              turret.setRelativeEncoderPosition(turretRezeroLocationRot);
            },
            turret)
        .ignoringDisable(true)
        .withName("RezeroTurret");
  }

  // #endregion
}
