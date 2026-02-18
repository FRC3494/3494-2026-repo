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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.VisionConstants.LimelightConstants;

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
    public static final int pdhCanId = 63;

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

    public static final int turretMotorCanId = 15;

    public static final int hoodMotorCanId = 20;

    public static final LimelightConstants[] aprilTagLimelights = {
      new LimelightConstants(
          "limelight-barge",
          new Pose3d(
              Inches.of(-8.1296),
              Inches.of(-6.376),
              Inches.of(27.6),
              new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(90)))),
      new LimelightConstants(
          "limelight-coral",
          new Pose3d(
              Inches.of(-8.0936),
              Inches.of(6.376),
              Inches.of(29.1),
              new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-90)))),
      new LimelightConstants(
          "limelight-left",
          new Pose3d(
              Meters.of(-0.2093),
              Meters.of(-0.2092),
              Meters.of(0.327),
              new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(90)))),
      new LimelightConstants(
          "limelight-swerve",
          new Pose3d(
              Meters.of(0.2355),
              Meters.of(-0.2499),
              Meters.of(0.2267),
              new Rotation3d(Degrees.of(0), Degrees.of(15), Degrees.of(105))))
    };
  }

  public static class OIConstants {
    public static final int primaryControllerPort = 0;
    public static final int leftButtonBoardPort = 1;
    public static final int rightButtonBoardPort = 2;

    public static final double controllerStickDeadband = 0.05;
  }

  public static class DriveConstants {
    /*
    ! Things that need to be configured in addition to AdvantageKit Swerve Template configs

    * maxAngularSpeedFactor
       - Units: rad/sec
       - Divide max rotation speed when driving by max rotation speed while stationary
    */

    public static final double maxSpeedMetersPerSec = 4.8;
    // * Max rotation speed (Rad/Sec) while moving / Max rotation speed while stationary
    public static final double maxAngularSpeedFactor = (1 / 1);
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth =
        Units.inchesToMeters(20.75); // Units.inchesToMeters(26.5);
    public static final double wheelBase =
        Units.inchesToMeters(20.75); // Units.inchesToMeters(26.5);
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
    public static final double wheelRadiusMeters =
        Units.inchesToMeters(
            1.99162835); // When using linear characterization: actual linear distance / wheel delta
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
    public static final double driveKp = 0.0000061024;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.17311;
    public static final double driveKv = 0.11345;
    public static final double driveKa = 0.011064;
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

    // Pigeon config
    public static final double pigeonGyroYawTrimDegPerRot = 6.558166667;

    // Auto config
    public static final boolean mirrorForRedAlliance = true;

    public static final double autoLinearKp = 10.0;
    public static final double autoLinearKi = 0.0;
    public static final double autoLinearKd = 0.0;

    public static final double autoAngularKp = 7.5;
    public static final double autoAngularKi = 0.0;
    public static final double autoAngularKd = 0.0;

    public static final Distance fieldWidth = Meters.of(8.07);
    public static final Distance fieldLength = Meters.of(16.54);

    public static class AutoAlignConstants {
      // Tuned with Ziegler-Nichols for classic PID
      // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
      // kU is 0.5, tU is 0.193s (9.65 robot loops)
      public static final double autoAlignLinearKp = 4.0;
      public static final double autoAlignLinearKi = 0;
      public static final double autoAlignLinearKd = 0.1;
      public static final Distance autoAlignLinearTolerance = Centimeters.of(1.0);

      public static final double autoAlignAngularKp = 5.0;
      public static final double autoAlignAngularKi = 0.0;
      public static final double autoAlignAngularKd = 0.1;

      public static final Pose2d climbPose =
          new Pose2d(Meters.of(1.7608400583267212), Meters.of(2.085599899291992), Rotation2d.kZero);
    }
  }

  public static final class VisionConstants {
    public static final Distance maxTagDistance = Meters.of(5.0);

    public static final double maxDistanceStdDev = 999999;
    public static final Angle maxAngleStdDev = Degrees.of(999999);

    public static final boolean useMegaTag2 = true;

    public static record LimelightConstants(String name, Pose3d position) {}
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

  public static final class TurretConstants {
    public static final boolean turretInverted = false;
    public static final int turretCurrentLimit = 20;

    public static final double turretKp = 0.0;
    public static final double turretKi = 0.0;
    public static final double turretKd = 0.0;

    public static final double turretKs = 0.0;
    public static final double turretKv = 0.0;
    public static final double turretKa = 0.0;

    public static final double turretGearRatio = (1.0 / 5.0) * (9.0 / 1.0);
  }

  public static final class HoodConstants {
    public static final boolean hoodInverted = false;
    public static final int hoodCurrentLimit = 20;

    public static final double hoodKp = 0.0;
    public static final double hoodKi = 0.0;
    public static final double hoodKd = 0.0;

    public static final double hoodKs = 0.0;
    public static final double hoodKv = 0.0;
    public static final double hoodKa = 0.0;

    public static final Rotation2d hoodMinAngle = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d hoodMaxAngle = Rotation2d.fromDegrees(0.0);
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
