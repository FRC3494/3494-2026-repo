package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ShooterConstants.HoodConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.AutoAlignConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.AimShooterMathLinear;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.SetFlywheelCommand;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.SetHoodCommand;
import frc.robot.subsystems.shooter.turret.SetTurretCommand;
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

  private final AimShooterMathLinear aimShooterMathLinear;

  // ==================== CLIMBER ====================
  private LoggedNetworkNumber climberUpPos =
      new LoggedNetworkNumber("Tunable/ClimberUpPos", climberMaxPosition);
  private LoggedNetworkNumber climberDownPos =
      new LoggedNetworkNumber("Tunable/ClimberDownPos", climberMinPosition);
  // How far DOWN the climber is for "mid" position
  public LoggedNetworkNumber climberMidFactor =
      new LoggedNetworkNumber("Tunable/ClimberMidFactor", 0.6);

  // ==================== HOPPER ====================
  private LoggedNetworkNumber spindexerSpeed = new LoggedNetworkNumber("Tunable/SpindexerRPM", 75);
  private LoggedNetworkNumber kickerSpeedFactor =
      new LoggedNetworkNumber(
          "Tunable/KickerSpeedFactor", 0.8); // Number to multiply flywheel speed by
  private boolean spindexerInverted = false;

  // ==================== INTAKE ====================
  private LoggedNetworkNumber intakeSpeed = new LoggedNetworkNumber("Tunable/IntakeRPM", 1100);

  // ==================== FLYWHEEL ====================
  private LoggedNetworkNumber flywheelThreshold =
      new LoggedNetworkNumber("Tunable/FlywheelThreshold", 0.95);
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

  public final Command setFlywheelCommand;
  public final Command setHoodCommand;
  public final Command setTurretCommand;

  public RobotCommands(
      Climber climber,
      Drive drive,
      Hopper hopper,
      Intake intake,
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      AimShooterMathLinear aimShooterMathLinear) {
    this.climber = climber;
    this.drive = drive;
    this.hopper = hopper;
    this.intake = intake;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;

    this.aimShooterMathLinear = aimShooterMathLinear;

    joystickDriveCommand =
        DriveCommands.joystickDrive(
            drive,
            OI.DriveOI::joystickDriveX,
            OI.DriveOI::joystickDriveY,
            OI.DriveOI::joystickDriveOmega);

    setFlywheelCommand =
        new SetFlywheelCommand(flywheel, () -> aimShooterMathLinear.getFlywheelSpeed());
    setHoodCommand = new SetHoodCommand(hood, aimShooterMathLinear::getHoodAngle);
    setTurretCommand = new SetTurretCommand(turret, aimShooterMathLinear::getTurretAngleRot);
  }

  // ==================== CLIMBER ====================
  public Command climberUp() {
    return runOnce(
        () -> {
          climber.setPosition(climberUpPos.get());
        },
        climber);
  }

  public Command climberMid() {
    return runOnce(
        () -> {
          climber.setPosition(climberDownPos.get() * climberMidFactor.get());
        },
        climber);
  }

  public Command climberDown() {
    return runOnce(
        () -> {
          climber.setPosition(climberDownPos.get());
        },
        climber);
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
              climber.setRelativeEncoderPosition(climberMinPosition);
              climber.setCurrentLimit(Amps.of(climberCurrentLimit));
            },
            climber));
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
        waitUntil(() -> climber.getPosition() <= climberMaxPosition + 0.05),
        stopClimber());
  }

  public Command climberManualDown() {
    return sequence(
        runOnce(
            () -> {
              climber.setOpenLoop(Volts.of(2));
            },
            climber),
        waitUntil(() -> climber.getPosition() >= climberMinPosition - 0.05),
        stopClimber());
  }

  // ==================== DRIVE ====================
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

  // ==================== HOPPER ====================
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

  public Command runKicker() {
    return runOnce(
        () -> {
          hopper.setKickerVelocity(RPM.of(flywheelSpeed.get() * kickerSpeedFactor.get()));
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

  // ==================== INTAKE ====================
  public Command intake() {
    return sequence(runIntake(), runSpindexerSlow());
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

  // ==================== SHOOTER ====================
  public Command shoot() {
    return sequence(
            runSpindexer(),
            startHood(),
            waitUntil(() -> flywheel.atVelocity(flywheelThreshold.get())),
            runOnce(
                () -> {
                  spindexerInverted = !spindexerInverted;
                }),
            runKicker(),
            runIntake())
        .withTimeout(3);
  }

  public Command spinDownFromShoot() {
    return sequence(
        stopSpindexer(),
        waitSeconds(0.25),
        stopKicker(),
        waitSeconds(0.25),
        stopIntake(),
        stopHood());
  }

  public Command enableAutoShooterSettings() {
    return runOnce(
        () -> {
          flywheel.setDefaultCommand(setFlywheelCommand);
          hood.setDefaultCommand(setHoodCommand);
        },
        flywheel,
        hood);
  }

  public Command shootClose() {
    return sequence(
            runFlywheelManual(RPM.of(3000)), setHoodManual(Rotation2d.fromDegrees(24.5)), shoot())
        .withTimeout(3);
  }

  public Command shootMedium() {
    return sequence(
        runFlywheelManual(RPM.of(3250)), setHoodManual(Rotation2d.fromDegrees(34.5)), shoot());
  }

  public Command shootFar() {
    return sequence(
        runFlywheelManual(RPM.of(4500)), setHoodManual(Rotation2d.fromDegrees(29.5)), shoot());
  }

  public Command shootNeutralZone() {
    return sequence(
        runFlywheelManual(RPM.of(4500)), setHoodManual(Rotation2d.fromDegrees(30.0)), shoot());
  }

  public Command shootDashboard() {
    return sequence(
        runFlywheelManual(() -> RPM.of(flywheelSpeed.get())),
        setHoodManual(() -> Rotation2d.fromDegrees(hoodAngle.get())),
        shoot());
  }

  // ==================== FLYWHEEL ====================
  public Command stopFlywheel() {
    return runOnce(
        () -> {
          flywheel.removeDefaultCommand();
          flywheel.setVelocity(RPM.of(0));
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

  // ==================== HOOD ====================
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
        .withTimeout(hoodRezeroTimeoutSeconds);
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

  // ==================== TURRET ====================
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
          turret.setPosition(Units.degreesToRotations(180));
        },
        turret);
  }

  public Command rezeroTurret() {
    return runOnce(
            () -> {
              turret.rezeroFromAbsEncoder();
            },
            turret)
        .ignoringDisable(true);
  }

  public Command enableAutoTurret() {
    return runOnce(
            () -> {
              turret.setDefaultCommand(setTurretCommand);
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
}
