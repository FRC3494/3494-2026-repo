package frc.robot;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RobotCommands {
  private final Climber climber;
  private final Drive drive;
  private final Hopper hopper;
  private final Intake intake;
  private final Shooter shooter;

  // ==================== CLIMBER ====================
  private LoggedNetworkNumber climberUpPower =
      new LoggedNetworkNumber("Tunable/ClimberUpPower", 0.25);
  private LoggedNetworkNumber climberDownPower =
      new LoggedNetworkNumber("Tunable/ClimberDownPower", 0.75);

  // ==================== HOPPER ====================
  private LoggedNetworkNumber spindexerSpeed =
      new LoggedNetworkNumber("Tunable/SpindexerVoltage", 8.0);
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
  private LoggedNetworkNumber hoodAngle = new LoggedNetworkNumber("Tunable/HoodAngle", 0.0);
  private LoggedNetworkNumber hoodIncrement =
      new LoggedNetworkNumber("Tunable/HoodIncrementDeg", 2.0);

  public RobotCommands(
      Climber climber, Drive drive, Hopper hopper, Intake intake, Shooter shooter) {
    this.climber = climber;
    this.drive = drive;
    this.hopper = hopper;
    this.intake = intake;
    this.shooter = shooter;
  }

  // ==================== CLIMBER ====================
  public Command runClimberUp() {
    return runOnce(
        () -> {
          climber.setPosition(Rotation2d.fromRotations(-climberUpPower.get()));
        },
        climber);
  }

  public Command runClimberDown() {
    return runOnce(
        () -> {
          climber.setPosition(Rotation2d.fromRotations(climberDownPower.get()));
        },
        climber);
  }

  public Command stopClimber() {
    return runOnce(
        () -> {
          climber.setPosition(Rotation2d.kZero);
        },
        climber);
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
          hopper.setSpindexerVelocity(RPM.of((spindexerInverted ? -1 : 1) * spindexerSpeed.get()));
        },
        hopper);
  }

  public Command runSpindexerReverse() {
    return runOnce(
        () -> {
          hopper.setSpindexerVelocity(RPM.of((spindexerInverted ? 1 : -1) * spindexerSpeed.get()));
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
          hoodAngle.set(0);
        });
  }

  public Command setTrenchShot() {
    return runOnce(
        () -> {
          flywheelSpeed.set(3250);
          flywheelThresholdSpeed.set(3200);
          hoodAngle.set(10);
        });
  }

  public Command setOutpostShot() {
    return runOnce(
        () -> {
          flywheelSpeed.set(4500);
          flywheelThresholdSpeed.set(4400);
          hoodAngle.set(5);
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
        waitUntil(() -> shooter.getFlywheelVelocity().gte(RPM.of(flywheelThresholdSpeed.get()))),
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
          shooter.setFlywheelVelocity(RPM.of(flywheelSpeed.get()));
        },
        shooter);
  }

  public Command stopFlywheel() {
    return runOnce(
        () -> {
          shooter.setFlywheelVelocity(RPM.of(0));
        },
        shooter);
  }

  // ==================== HOOD ====================
  public Command increaseHoodAngle() {
    return run(
        () -> {
          shooter.setHoodAngle(
              shooter.getHoodSetpoint().plus(Rotation2d.fromDegrees(hoodIncrement.get())));
        },
        shooter);
  }

  public Command decreaseHoodAngle() {
    return run(
        () -> {
          shooter.setHoodAngle(
              shooter.getHoodSetpoint().minus(Rotation2d.fromDegrees(hoodIncrement.get())));
        },
        shooter);
  }

  public Command hoodUp() {
    return runOnce(
        () -> {
          shooter.setHoodAngle(Rotation2d.fromDegrees(hoodAngle.get()));
        },
        shooter);
  }

  public Command hoodDown() {
    return runOnce(
        () -> {
          shooter.setHoodAngle(Rotation2d.kZero);
        },
        shooter);
  }
}
