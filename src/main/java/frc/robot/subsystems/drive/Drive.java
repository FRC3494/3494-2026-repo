// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.OI;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private final SysIdRoutine driveSysId;
  private final SysIdRoutine turnSysId;
  private final SysIdRoutine robotTurnSysId;

  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final Field2d robotField2d = new Field2d();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  // TODO: fix this to fix init yaw?
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  @Getter @Setter @AutoLogOutput private boolean autoAligning = false;

  private final PIDController xController =
      new PIDController(autoLinearKp, autoLinearKi, autoLinearKd);
  private final PIDController yController =
      new PIDController(autoLinearKp, autoLinearKi, autoLinearKd);
  private final PIDController headingController =
      new PIDController(autoAngularKp, autoAngularKi, autoAngularKd);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    driveSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/DriveSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDriveCharacterization(voltage.in(Volts)), null, this));
    turnSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/TurnSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runTurnCharacterization(voltage.in(Volts)), null, this));
    robotTurnSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/RobotTurnSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runRobotTurnCharacterization(voltage.in(Volts)), null, this));

    SmartDashboard.putData("RobotField", robotField2d);
    SmartDashboard.putData("Drive", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "Max Drive Speed (FtPerSec)",
        () -> Units.metersToFeet(maxSpeedMetersPerSec),
        (double value) -> maxSpeedMetersPerSec = Units.feetToMeters(value));
    builder.addDoubleProperty(
        "Max Angular Speed (DegPerSec)",
        () -> Units.radiansToDegrees(maxAngularSpeedRadPerSec),
        (double value) -> maxAngularSpeedRadPerSec = Units.degreesToRadians(value));

    builder.addDoubleProperty(
        "Max Shooting Drive Speed (FtPerSec)",
        () -> Units.metersToFeet(maxShootingSpeedMetersPerSec),
        (double value) -> maxShootingSpeedMetersPerSec = Units.feetToMeters(value));
    builder.addDoubleProperty(
        "Max Shooting Angular Speed (DegPerSec)",
        () -> Units.radiansToDegrees(maxShootingAngularSpeedRadPerSec),
        (double value) -> maxShootingAngularSpeedRadPerSec = Units.degreesToRadians(value));

    builder.addDoubleArrayProperty(
        "Auto Linear PID",
        () -> new double[] {autoLinearKp, autoLinearKi, autoLinearKd},
        (double[] values) -> setAutoLinearPID(values[0], values[1], values[2]));
    builder.addDoubleArrayProperty(
        "Auto Angular PID",
        () -> new double[] {autoAngularKp, autoAngularKi, autoAngularKd},
        (double[] values) -> setAutoAngularPID(values[0], values[1], values[2]));

    builder.addDoubleArrayProperty(
        "Drive PID",
        () -> new double[] {driveKp, driveKi, driveKd},
        (double[] values) -> setDrivePID(values[0], values[1], values[2]));
    builder.addDoubleArrayProperty(
        "Drive SVA",
        () -> new double[] {driveKs, driveKv, driveKa},
        (double[] values) -> setDriveSVA(values[0], values[1], values[2]));
    builder.addDoubleArrayProperty(
        "Turn PID",
        () -> new double[] {turnKp, turnKi, turnKd},
        (double[] values) -> setTurnPID(values[0], values[1], values[2]));

    builder.addDoubleProperty(
        "AutoAlign/Trench/XTolerance (ft)",
        () -> trenchXTolerance.in(Feet),
        (double value) -> trenchXTolerance = Feet.of(value));
    builder.addDoubleProperty(
        "AutoAlign/Trench/YTolerance (ft)",
        () -> trenchYTolerance.in(Feet),
        (double value) -> trenchYTolerance = Feet.of(value));
    builder.addDoubleProperty(
        "AutoAlign/Trench/AngularTolerance (deg)",
        () -> trenchAngularTolerance.getDegrees(),
        (double value) -> trenchAngularTolerance = Rotation2d.fromDegrees(value));

    builder.addDoubleProperty(
        "AutoAlign/Trench/OppositeTrenchOffset (ft)",
        () -> closerToOppositeTrenchLine.minus(fieldLength.div(2.0)).in(Feet),
        (double value) -> closerToOppositeTrenchLine = fieldLength.div(2.0).plus(Feet.of(value)));
    builder.addDoubleProperty(
        "AutoAlign/Trench/PreTrenchOffset (ft)",
        () -> preTrenchOffset.in(Feet),
        (double value) -> preTrenchOffset = Feet.of(value));
    builder.addDoubleProperty(
        "AutoAlign/Trench/PostTrenchOffset (ft)",
        () -> postTrenchOffset.in(Feet),
        (double value) -> postTrenchOffset = Feet.of(value));
  }

  private void logSendableValues() {
    Logger.recordOutput("Drive/MaxDriveSpeed", MetersPerSecond.of(maxSpeedMetersPerSec));
    Logger.recordOutput("Drive/MaxAngularSpeed", RadiansPerSecond.of(maxAngularSpeedRadPerSec));

    Logger.recordOutput(
        "Drive/MaxShootingDriveSpeed", MetersPerSecond.of(maxShootingSpeedMetersPerSec));
    Logger.recordOutput(
        "Drive/MaxShootingAngularSpeed", RadiansPerSecond.of(maxShootingAngularSpeedRadPerSec));

    Logger.recordOutput("Drive/AutoLinearPID/kP", autoLinearKp);
    Logger.recordOutput("Drive/AutoLinearPID/kI", autoLinearKi);
    Logger.recordOutput("Drive/AutoLinearPID/kD", autoLinearKd);
    Logger.recordOutput("Drive/AutoAngularPID/kP", autoAngularKp);
    Logger.recordOutput("Drive/AutoAngularPID/kI", autoAngularKi);
    Logger.recordOutput("Drive/AutoAngularPID/kD", autoAngularKd);

    Logger.recordOutput("Drive/DrivePID/kP", driveKp);
    Logger.recordOutput("Drive/DrivePID/kI", driveKi);
    Logger.recordOutput("Drive/DrivePID/kD", driveKd);
    Logger.recordOutput("Drive/DrivePID/kS", driveKs);
    Logger.recordOutput("Drive/DrivePID/kV", driveKv);
    Logger.recordOutput("Drive/DrivePID/kA", driveKa);

    Logger.recordOutput("Drive/TurnPID/kP", turnKp);
    Logger.recordOutput("Drive/TurnPID/kI", turnKi);
    Logger.recordOutput("Drive/TurnPID/kD", turnKd);

    Logger.recordOutput("Drive/AutoAlign/Trench/XTolerance", trenchXTolerance);
    Logger.recordOutput("Drive/AutoAlign/Trench/YTolerance", trenchYTolerance);
    Logger.recordOutput("Drive/AutoAlign/Trench/AngularTolerance", trenchAngularTolerance);

    Logger.recordOutput(
        "Drive/AutoAlign/Trench/CloserToOppositeTrenchLine", closerToOppositeTrenchLine);
    Logger.recordOutput("Drive/AutoAlign/Trench/PreTrenchOffset", preTrenchOffset);
    Logger.recordOutput("Drive/AutoAlign/Trench/PostTrenchOffset", postTrenchOffset);
  }

  // @codescene (disable: "Bumpy Road Ahead", disable: "Complex Method")
  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    logSendableValues();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void followTrajectory(SwerveSample sample) {
    Logger.recordOutput("Choreo/TargetPose", sample.getPose());
    Logger.recordOutput(
        "Choreo/TargetVelocity",
        new Pose2d(
            Meters.of(sample.x + sample.vx),
            Meters.of(sample.y + sample.vy),
            Rotation2d.fromRadians(sample.heading + sample.omega)));
    Logger.recordOutput(
        "Choreo/TargetAccel",
        new Pose2d(
            Meters.of(sample.x + sample.ax),
            Meters.of(sample.y + sample.ay),
            Rotation2d.fromRadians(sample.heading + sample.alpha)));
    Logger.recordOutput("Choreo/t", Seconds.of(sample.t));
    Logger.recordOutput("Choreo/ModuleForcesX", sample.moduleForcesX());
    Logger.recordOutput("Choreo/ModuleForcesY", sample.moduleForcesY());

    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading),
            getRotation());

    Logger.recordOutput(
        "Choreo/VelocitySetpoint",
        new Pose2d(
            pose.getX() + sample.vx + xController.calculate(pose.getX(), sample.x),
            pose.getY() + sample.vy + yController.calculate(pose.getY(), sample.y),
            Rotation2d.fromRadians(
                pose.getRotation().getRadians()
                    + sample.omega
                    + headingController.calculate(
                        pose.getRotation().getRadians(), sample.heading))));

    // Apply the generated speeds
    runVelocity(speeds);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runDriveCharacterization(double output) {
    for (var module : modules) {
      module.runDriveCharacterization(output, Rotation2d.kZero);
    }
  }

  public void runTurnCharacterization(double output) {
    for (var module : modules) {
      module.runTurnCharacterization(output);
    }
  }

  /** Spins robot in place with the specified drive output */
  public void runRobotTurnCharacterization(double output) {
    modules[0].runDriveCharacterization(output, Rotation2d.fromDegrees(135));
    modules[1].runDriveCharacterization(output, Rotation2d.fromDegrees(45));
    modules[2].runDriveCharacterization(output, Rotation2d.fromDegrees(225));
    modules[3].runDriveCharacterization(output, Rotation2d.fromDegrees(315));
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command driveSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runDriveCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(driveSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command driveSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runDriveCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(driveSysId.dynamic(direction));
  }

  public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runTurnCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(turnSysId.quasistatic(direction));
  }

  public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runTurnCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(turnSysId.dynamic(direction));
  }

  public Command robotTurnSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runRobotTurnCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(robotTurnSysId.quasistatic(direction));
  }

  public Command robotTurnSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runRobotTurnCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(robotTurnSysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private void setAutoLinearPID(double p, double i, double d) {
    autoLinearKp = p;
    autoLinearKi = i;
    autoLinearKd = d;
    xController.setPID(p, i, d);
    yController.setPID(p, i, d);
  }

  private void setAutoAngularPID(double p, double i, double d) {
    autoAngularKp = p;
    autoAngularKi = i;
    autoAngularKd = d;
    headingController.setPID(p, i, d);
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    var pose = poseEstimator.getEstimatedPosition();
    robotField2d.setRobotPose(pose);
    return pose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public Rotation2d getRawRotation() {
    return rawGyroRotation;
  }

  public double getYawVelocityRadPerSec() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);

    // setRotation(pose.getRotation());
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    switch (Constants.driveMode) {
      case DEMO -> {
        return !OI.ShooterOI.shoot().getAsBoolean()
            ? maxSpeedMetersPerSec * demoModeSpeedFactor
            : maxShootingSpeedMetersPerSec * demoModeSpeedFactor;
      }
      default -> {
        return !OI.ShooterOI.shoot().getAsBoolean()
            ? maxSpeedMetersPerSec
            : maxShootingSpeedMetersPerSec;
      }
    }
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    switch (Constants.driveMode) {
      case DEMO -> {
        return !OI.ShooterOI.shoot().getAsBoolean()
            ? maxAngularSpeedRadPerSec * demoModeSpeedFactor
            : maxShootingAngularSpeedRadPerSec * demoModeSpeedFactor;
      }
      default -> {
        return !OI.ShooterOI.shoot().getAsBoolean()
            ? maxAngularSpeedRadPerSec
            : maxShootingAngularSpeedRadPerSec;
      }
    }
  }

  public void rezeroTurnEncoders() {
    for (var module : modules) {
      module.rezeroTurnEncoder();
    }
  }

  public void setDrivePID(double p, double i, double d) {
    for (var module : modules) {
      module.setDrivePID(p, i, d);
    }
  }

  public void setDriveSVA(double s, double v, double a) {
    for (var module : modules) {
      module.setDriveSVA(s, v, a);
    }
  }

  public void setTurnPID(double p, double i, double d) {
    for (var module : modules) {
      module.setTurnPID(p, i, d);
    }
  }

  public boolean isGyroConnected() {
    return gyroInputs.connected;
  }

  public void setRotation(Rotation2d rotation) {
    gyroIO.setYaw(rotation);
    poseEstimator.resetRotation(rotation);
  }

  public void resetYaw() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      setRotation(Rotation2d.k180deg);
    } else {
      setRotation(Rotation2d.kZero);
    }
  }
}
