package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.Constants.ClimberConstants.climberMaxPosition;
import static frc.robot.Constants.ClimberConstants.climberMinPosition;
import static frc.robot.Constants.ShooterConstants.HoodConstants.hoodMinAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RobotCommands {
  private final Climber climber;
  private final Drive drive;
  private final Hopper hopper;
  private final Intake intake;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;

  // ==================== CLIMBER ====================
  private LoggedNetworkNumber climberUpPos =
      new LoggedNetworkNumber("Tunable/ClimberUpPos", climberMaxPosition);
  private LoggedNetworkNumber climberDownPos =
      new LoggedNetworkNumber("Tunable/ClimberDownPos", climberMinPosition);

  // ==================== HOPPER ====================
  private LoggedNetworkNumber spindexerSpeed = new LoggedNetworkNumber("Tunable/SpindexerRPM", 100);
  private LoggedNetworkNumber feederSpeedFactor =
      new LoggedNetworkNumber(
          "Tunable/FeederSpeedFactor", 0.60); // Number to multiply flywheel speed by
  private boolean spindexerInverted = false;

  // ==================== INTAKE ====================
  private LoggedNetworkNumber intakeSpeed = new LoggedNetworkNumber("Tunable/IntakeRPM", 3500);

  // ==================== FLYWHEEL ====================
  private LoggedNetworkNumber flywheelThresholdSpeed =
      new LoggedNetworkNumber("Tunable/FlywheelThresholdRPM", 2900);
  private LoggedNetworkNumber flywheelSpeed = new LoggedNetworkNumber("Tunable/FlywheelRPM", 3000);

  // ==================== HOOD ====================
  private LoggedNetworkNumber hoodAngle =
      new LoggedNetworkNumber("Tunable/HoodAngle", 0.0 + hoodMinAngle.getDegrees());
  private LoggedNetworkNumber hoodIncrement =
      new LoggedNetworkNumber("Tunable/HoodIncrementDeg", 2.0);

  public RobotCommands(
      Climber climber,
      Drive drive,
      Hopper hopper,
      Intake intake,
      Flywheel flywheel,
      Hood hood,
      Turret turret) {
    this.climber = climber;
    this.drive = drive;
    this.hopper = hopper;
    this.intake = intake;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
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
          climber.setPosition(climberDownPos.get() / 2.0);
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

  // ==================== INTAKE ====================
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

  // ==================== HOPPER ====================
  public Command runSpindexer() {
    return runOnce(
        () -> {
          hopper.setSpindexerVelocity(RPM.of((spindexerInverted ? 1 : 1) * spindexerSpeed.get()));
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

  public Command runFeeder() {
    return runOnce(
        () -> {
          hopper.setFeederVelocity(RPM.of(flywheelSpeed.get() * feederSpeedFactor.get()));
        },
        hopper);
  }

  public Command stopFeeder() {
    return runOnce(
        () -> {
          hopper.setFeederVelocity(RPM.of(0.0));
        },
        hopper);
  }

  // ==================== SHOOTER ====================
  public Command setHubShot() {
    return runOnce(
        () -> {
          flywheelSpeed.set(3000);
          flywheelThresholdSpeed.set(2900);
          hoodAngle.set(0 + hoodMinAngle.getDegrees());
        });
  }

  public Command setTrenchShot() {
    return runOnce(
        () -> {
          flywheelSpeed.set(3250);
          flywheelThresholdSpeed.set(3200);
          hoodAngle.set(10 + hoodMinAngle.getDegrees());
        });
  }

  public Command setOutpostShot() {
    return runOnce(
        () -> {
          flywheelSpeed.set(4500);
          flywheelThresholdSpeed.set(4400);
          hoodAngle.set(5 + hoodMinAngle.getDegrees());
        });
  }

  public Command setNeutralZoneShot() {
    return runOnce(
        () -> {
          flywheelSpeed.set(4900);
          flywheelThresholdSpeed.set(5000);
          hoodAngle.set(30);
        });
  }

  public Command shoot() {
    return sequence(
        runFlywheel(),
        hoodUp(),
        waitUntil(() -> flywheel.getVelocity().gte(RPM.of(flywheelThresholdSpeed.get()))),
        runOnce(
            () -> {
              spindexerInverted = !spindexerInverted;
            }),
        runSpindexer(),
        runFeeder(),
        runIntake());
  }

  public Command spinDownFromShoot() {
    return sequence(
        stopSpindexer(),
        waitSeconds(0.25),
        stopFeeder(),
        waitSeconds(0.25),
        stopFlywheel(),
        stopIntake());
  }

  // ==================== FLYWHEEL ====================
  public Command runFlywheel() {
    return runOnce(
        () -> {
          flywheel.setVelocity(RPM.of(flywheelSpeed.get()));
        },
        flywheel);
  }

  public Command stopFlywheel() {
    return runOnce(
        () -> {
          flywheel.setVelocity(RPM.of(0));
        },
        flywheel);
  }

  // ==================== HOOD ====================
  public Command increaseHoodAngle() {
    return run(
        () -> {
          hood.setPosition(
              hood.getHoodSetpoint().plus(Rotation2d.fromDegrees(hoodIncrement.get())));
        },
        hood);
  }

  public Command decreaseHoodAngle() {
    return run(
        () -> {
          hood.setPosition(
              hood.getHoodSetpoint().minus(Rotation2d.fromDegrees(hoodIncrement.get())));
        },
        hood);
  }

  public Command hoodUp() {
    return runOnce(
        () -> {
          hood.setPosition(Rotation2d.fromDegrees(hoodAngle.get()));
        },
        hood);
  }

  public Command hoodDown() {
    return runOnce(
        () -> {
          hood.setPosition(Rotation2d.kZero);
        },
        hood);
  }
}
