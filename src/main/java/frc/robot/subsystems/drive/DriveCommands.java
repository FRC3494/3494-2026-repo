// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.5; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return startRun(
            () -> {
              drive.setAutoAligning(false);
            },
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .withName("JoystickDrive");
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .withName("JoystickDriveAtAngle");
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return sequence(
            // Reset data
            runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                }),

            // Allow modules to orient
            run(
                    () -> {
                      drive.runDriveCharacterization(0.0);
                    },
                    drive)
                .withTimeout(FF_START_DELAY),

            // Start timer
            runOnce(timer::restart),

            // Accelerate and gather data
            run(
                    () -> {
                      double voltage = timer.get() * FF_RAMP_RATE;
                      drive.runDriveCharacterization(voltage);
                      velocitySamples.add(drive.getFFCharacterizationVelocity());
                      voltageSamples.add(voltage);
                    },
                    drive)

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      int n = velocitySamples.size();
                      double sumX = 0.0;
                      double sumY = 0.0;
                      double sumXY = 0.0;
                      double sumX2 = 0.0;
                      for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                      }
                      double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                      double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                      NumberFormat formatter = new DecimalFormat("#0.00000000");
                      System.out.println("********** Drive FF Characterization Results **********");
                      System.out.println("\tkS: " + formatter.format(kS));
                      System.out.println("\tkV: " + formatter.format(kV));
                    }))
        .withName("DriveFFCharacterization");
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return parallel(
            // Drive control sequence
            sequence(
                // Reset acceleration limiter
                runOnce(
                    () -> {
                      limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                run(
                    () -> {
                      double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                    },
                    drive)),

            // Measurement sequence
            sequence(
                // Wait for modules to fully orient before starting measurement
                waitSeconds(1.0),

                // Record starting measurement
                runOnce(
                    () -> {
                      state.positions = drive.getWheelRadiusCharacterizationPositions();
                      state.lastAngle = drive.getRotation();
                      state.gyroDelta = 0.0;
                    }),

                // Update gyro delta
                run(() -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                    // When cancelled, calculate and print results
                    .finallyDo(
                        () -> {
                          double[] positions = drive.getWheelRadiusCharacterizationPositions();
                          double wheelDelta = 0.0;
                          for (int i = 0; i < 4; i++) {
                            wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                          }
                          double wheelRadius =
                              (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                          NumberFormat formatter = new DecimalFormat("#0.00000000");
                          System.out.println(
                              "********** Wheel Radius Characterization Results **********");
                          System.out.println(
                              "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                          System.out.println(
                              "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                          System.out.println(
                              "\tWheel Radius: "
                                  + formatter.format(wheelRadius)
                                  + " meters, "
                                  + formatter.format(Units.metersToInches(wheelRadius))
                                  + " inches");
                        })))
        .withName("WheelRadiusRotationalCharacterization");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  public static Command linearWheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    LinearWheelRadiusCharacterizationState state = new LinearWheelRadiusCharacterizationState();

    return parallel(
            // Drive control sequence
            sequence(
                // Reset acceleration limiter
                runOnce(
                    () -> {
                      limiter.reset(0.0);
                    }),

                // Turn in place, accelerating up to full speed
                run(
                    () -> {
                      double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                      drive.runVelocity(new ChassisSpeeds(speed, 0.0, 0.0));
                    },
                    drive)),

            // Measurement sequence
            sequence(
                // Wait for modules to fully orient before starting measurement
                waitSeconds(0.0),

                // Record starting measurement
                runOnce(
                    () -> {
                      state.positions = drive.getWheelRadiusCharacterizationPositions();
                    })))

        // When cancelled, calculate and print results
        .finallyDo(
            () -> {
              double[] positions = drive.getWheelRadiusCharacterizationPositions();
              double wheelDelta = 0.0;
              for (int i = 0; i < 4; i++) {
                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
              }

              NumberFormat formatter = new DecimalFormat("#0.000000");
              System.out.println(
                  "********** Linear Wheel Radius Characterization Results **********");
              System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
            })
        .withName("WheelRadiusLinearCharacterization");
  }

  private static class LinearWheelRadiusCharacterizationState {
    double[] positions = new double[4];
  }

  // @codescene (disable: "Large Method")
  public static Command turnSpeedCharacterization(Drive drive) {
    TurnSpeedCharacterizationState state = new TurnSpeedCharacterizationState();

    double ROTATION_SPEED_MARGIN = 3;

    return sequence(
            // Wait until spinning at max speed
            deadline(
                waitUntil(
                    () ->
                        Math.abs(
                                drive.getChassisSpeeds().omegaRadiansPerSecond
                                    - drive.getMaxAngularSpeedRadPerSec())
                            < ROTATION_SPEED_MARGIN),
                run(
                    () -> {
                      drive.runVelocity(
                          new ChassisSpeeds(0.0, 0.0, drive.getMaxAngularSpeedRadPerSec()));
                    },
                    drive)),

            // Measure rotation speed of robot while moving
            sequence(
                race(
                    waitSeconds(2.5),
                    run(
                        () -> {
                          drive.runVelocity(
                              ChassisSpeeds.fromFieldRelativeSpeeds(
                                  drive.getMaxLinearSpeedMetersPerSec(),
                                  0,
                                  drive.getMaxAngularSpeedRadPerSec(),
                                  drive.getRotation()));
                        },
                        drive),
                    run(
                        () -> {
                          state.movingRotationSpeeds.add(drive.getYawVelocityRadPerSec());
                        }))),

            // Wait until spinning in place at max speed
            deadline(
                waitUntil(
                    () ->
                        Math.abs(
                                drive.getChassisSpeeds().omegaRadiansPerSecond
                                    - drive.getMaxAngularSpeedRadPerSec())
                            < ROTATION_SPEED_MARGIN),
                run(
                    () -> {
                      drive.runVelocity(
                          new ChassisSpeeds(0.0, 0.0, drive.getMaxAngularSpeedRadPerSec()));
                    },
                    drive)),

            // Measure rotation speed of robot while stationary
            race(
                    waitSeconds(5),
                    run(
                        () -> {
                          drive.runVelocity(
                              new ChassisSpeeds(0.0, 0.0, drive.getMaxAngularSpeedRadPerSec()));
                        },
                        drive),
                    run(
                        () -> {
                          state.stationaryRotationSpeeds.add(drive.getYawVelocityRadPerSec());
                        }))
                .finallyDo(
                    () -> {
                      drive.stop();
                    }),

            // Stop driving and output values
            runOnce(
                () -> {
                  double movingAverageSpeed = 0.0;
                  for (double speed : state.movingRotationSpeeds) {
                    movingAverageSpeed += speed;
                  }
                  movingAverageSpeed /= state.movingRotationSpeeds.size();

                  double stationaryAverageSpeed = 0.0;
                  for (double speed : state.stationaryRotationSpeeds) {
                    stationaryAverageSpeed += speed;
                  }
                  stationaryAverageSpeed /= state.stationaryRotationSpeeds.size();

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Turn Speed Characterization Results **********");
                  System.out.println(
                      "\tRotation speed while moving: " + formatter.format(movingAverageSpeed));
                  System.out.println(
                      "\tRotation speed while stationary: "
                          + formatter.format(stationaryAverageSpeed));
                },
                drive))
        .withName("TurnSpeedCharacterization");
  }

  private static class TurnSpeedCharacterizationState {
    ArrayList<Double> movingRotationSpeeds = new ArrayList<Double>();
    ArrayList<Double> stationaryRotationSpeeds = new ArrayList<Double>();
  }

  public static Command turnErrorCharacterization(Drive drive) {
    return deadline(
            new WaitCommand(60),
            run(
                () -> {
                  drive.runVelocity(
                      new ChassisSpeeds(0, 0, 0.8 * drive.getMaxAngularSpeedRadPerSec()));
                },
                drive))
        .finallyDo(
            () -> {
              System.out.println(
                  "Complete turns: "
                      + Math.floor(drive.getRawRotation().getRadians() / (2 * Math.PI)));
            })
        .withName("TurnErrorCharacterization");
  }
}
