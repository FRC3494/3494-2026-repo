// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final DriveMode driveMode = DriveMode.NORMAL;

  public static enum DriveMode {
    NORMAL,
    DEMO
  };

  public static enum ElasticTab {
    Teleoperated,
    Autonomous,
    Testing
  }

  public static class RobotMap {
    public static final int pigeonCanId = 52;

    public static final int frontLeftDriveCanId = 18;
    public static final int frontRightDriveCanId = 19;
    public static final int backLeftDriveCanId = 30;
    public static final int backRightDriveCanId = 1;

    public static final int frontLeftTurnCanId = 16;
    public static final int frontRightTurnCanId = 17;
    public static final int backLeftTurnCanId = 2;
    public static final int backRightTurnCanId = 3;

    public static final int frontLeftAbsEncoderCanId = 3;
    public static final int frontRightAbsEncoderCanId = 2;
    public static final int backLeftAbsEncoderCanId = 1;
    public static final int backRightAbsEncoderCanId = 0;

    public static final int shooterLeftCanId = 10;
    public static final int shooterRightCanId = 11;

    public static final int hopperSpindexerCanId = 12;
    public static final int hopperFeederCanId = 13;

    public static final String[] aprilTagLimelightNames = {
      "limelight-right", "limelight-left", "limelight-swerve", "limelight-barge", "limelight-coral"
    };
  }

  public static class OIConstants {
    public static final int primaryControllerPort = 0;
    public static final int leftButtonBoardPort = 1;
    public static final int rightButtonBoardPort = 2;
  }

  public static class DriveConstants {
    /*
    ! Things that need to be configured in addition to AdvantageKit Swerve Template configs

    * maxAngularSpeedFactor
       - Units: rad/sec
       - Divide max rotation speed when driving by max rotation speed while stationary

    * pigeonYawPositionFactor
       - Units: Unit: rad
       - Divide measured yaw from Pigeon after 30 turns by expected reading after 30 turns
    */

    public static final double maxSpeedMetersPerSec = 4.8;
    // * Max rotation speed (Rad/Sec) while moving / Max rotation speed while stationary
    public static final double maxAngularSpeedFactor = (1 / 1);
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = 0.5222; // Units.inchesToMeters(26.5);
    public static final double wheelBase = 0.574675; // Units.inchesToMeters(26.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromRadians(4.391);
    public static final Rotation2d frontRightZeroRotation = Rotation2d.fromRadians(5.787);
    public static final Rotation2d backLeftZeroRotation = Rotation2d.fromRadians(3.397);
    public static final Rotation2d backRightZeroRotation = Rotation2d.fromRadians(5.934);

    // Drive motor configuration
    public static final boolean driveInverted = true;
    public static final int driveMotorCurrentLimit = 50;
    public static final double wheelRadiusMeters = Units.feetToMeters(27.25) / 126.411703;
    public static final double driveMotorReduction =
        (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // Mk4i L2 Gearing
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
    // Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = ((150.0 / 7.0) / (2.0 * Math.PI)); // Mk4i
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnAbsEncoderInverted = false;
    public static final boolean turnRelEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // Pigeon configuration
    // * Measured yaw (rad) after # of turns / # of turns
    public static final double pigeonYawPositionFactor =
        ((43 * 2 * Math.PI + 2.33861709845736) / (43 * 2 * Math.PI));

    // PathPlanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                wheelCOF,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);

    // Auto config
    public static final double autoLinearKp = 10.0;
    public static final double autoAngularKp = 7.5;
  }

  public static final class VisionConstants {
    public static final Distance maxTagDistance = Meters.of(5.0);

    public static final double maxDistanceStdDev = 998999;
    public static final Angle maxAngleStdDev = Degrees.of(999999);
  }

  public static final class ShooterConstants {
    public static final boolean shooterInverted = false;
    public static final int shooterCurrentLimit = 20;

    public static final double shooterKp = 0.0;
    public static final double shooterKi = 0.0;
    public static final double shooterKd = 0.0;

    public static final double shooterKs = 0.0;
    public static final double shooterKv = 0.0;
    public static final double shooterKa = 0.0;
  }

  public static final class HopperConstants {
    // spindexer constants
    public static final boolean hopperSpindexerInverted = false;
    public static final int hopperSpindexerCurrentLimit = 20;

    public static final double spindexerKp = 0.0;
    public static final double spindexerKi = 0.0;
    public static final double spindexerKd = 0.0;

    // spindexer sva constants
    // public static final double spindexerKs = 0.0;
    // public static final double spindexerKv = 0.0;
    // public static final double spindexerKa = 0.0;

    // feeder constants
    public static final boolean hopperFeederInverted = true;
    public static final int hopperFeederCurrentLimit = 20;

    public static final double feederKp = 0.0;
    public static final double feederKi = 0.0;
    public static final double feederKd = 0.0;

    // feeder sva constants
    // public static final double feederKs = 0.0;
    // public static final double feederKv = 0.0;
    // public static final double feederKa = 0.0;
  }
}
