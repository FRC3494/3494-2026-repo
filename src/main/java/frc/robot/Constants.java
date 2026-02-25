// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.fieldWidth;

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
    public static final int pdhCanId = 31;
    public static final int mitoCANdria1CanId = 8;
    public static final int mitoCANdria2CanId = 9;

    public static final class Climber {
      public static final int climberMotorCanId = 13;
    }

    public static final class Drive {
      public static final int pigeonCanId = 20;

      public static final int frontLeftDriveCanId = 1;
      public static final int frontRightDriveCanId = 18;
      public static final int backLeftDriveCanId = 30;
      public static final int backRightDriveCanId = 19;

      public static final int frontLeftTurnCanId = 3;
      public static final int frontRightTurnCanId = 16;
      public static final int backLeftTurnCanId = 2;
      public static final int backRightTurnCanId = 17;

      public static final int frontLeftAbsEncoderCanId = 0;
      public static final int frontRightAbsEncoderCanId = 1;
      public static final int backLeftAbsEncoderCanId = 2;
      public static final int backRightAbsEncoderCanId = 3;
    }

    public static final class Hopper {
      public static final int spindexerCanId = 6;
      public static final int feederCanId = 15;
    }

    public static final class Intake {
      public static final int spinnySpinnyCanId = 12;
      public static final int uppyDownyCanId = 11;

      public static final int intakeMagSensorDIO = 0;
    }

    public static final class Shooter {
      public static final int flywheelLeftCanId = 4;
      public static final int flywheelRightCanId = 5;

      public static final int hoodMotorCanId = 7;

      public static final int turretMotorCanId = 14;
      public static final int turretMagSensorDIO = 0;
    }

    public static final class Vision {
      public static final LimelightConstants[] aprilTagLimelights = {
        // * Left side
        new LimelightConstants(
            "limelight-square",
            new Pose3d(
                Inches.of(-10.0), // -0.254 m
                Inches.of(-13.239), // -0.3363 m
                Inches.of(8.843), // 0.2246 m
                new Rotation3d(Degrees.of(0), Degrees.of(20), Degrees.of(90)))),
        // * Left back
        new LimelightConstants(
            "limelight-kvale",
            new Pose3d(
                Inches.of(-13.239), // -0.3363 m
                Inches.of(-8.625), // -0.2191 m
                Inches.of(8.843), // 0.2246 m
                new Rotation3d(Degrees.of(0), Degrees.of(20), Degrees.of(180)))),
        // * Right side
        new LimelightConstants(
            "limelight-cube",
            new Pose3d(
                Inches.of(-10.0), // -0.254 m
                Inches.of(13.239), // 0.3363 m
                Inches.of(8.843), // 0.2246 m
                new Rotation3d(Degrees.of(0), Degrees.of(20), Degrees.of(-90)))),
        // * Right back
        new LimelightConstants(
            "limelight-angle",
            new Pose3d(
                Inches.of(-13.239), // -0.3363 m
                Inches.of(8.625), // 0.2191 m
                Inches.of(8.843), // 0.2246 m
                new Rotation3d(Degrees.of(0), Degrees.of(20), Degrees.of(180))))
      };
    }
  }

  public static class OIConstants {
    public static final int primaryControllerPort = 0;
    public static final int leftButtonBoardPort = 1;
    public static final int rightButtonBoardPort = 2;

    public static final double controllerStickDeadband = 0.05;
    public static final double controllerTriggerDeadband = 0.05;
  }

  // ========================= SUBSYSTEMS ========================= //

  public static class ClimberConstants {
    public static final boolean climberInverted = false;
    public static final int climberCurrentLimit = 70;

    public static final double climberKp = 0.0;
    public static final double climberKi = 0.0;
    public static final double climberKd = 0.0;

    public static final double climberKs = 0.0;
    public static final double climberKv = 0.0;
    public static final double climberKa = 0.0;
  }

  public static class DriveConstants {
    /*
    ! Things that need to be configured in addition to AdvantageKit Swerve Template configs

    * maxAngularSpeedFactor
       - Units: rad/sec
       - Divide max rotation speed when driving by max rotation speed while stationary
    */

    public static final double maxSpeedMetersPerSec = 4.56; // 14.961 ft/s
    // * Max rotation speed (Rad/Sec) while moving / Max rotation speed while stationary
    public static final double maxAngularSpeedFactor = (321.5229038 / 630.028839);
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(21.75);
    public static final double wheelBase = Units.inchesToMeters(21.75);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromRadians(5.515);
    public static final Rotation2d frontRightZeroRotation = Rotation2d.fromRadians(3.068);
    public static final Rotation2d backLeftZeroRotation = Rotation2d.fromRadians(0.417);
    public static final Rotation2d backRightZeroRotation = Rotation2d.fromRadians(1.974);

    // Drive motor configuration
    public static final boolean driveInverted = false;
    public static final int driveMotorCurrentLimit = 40;
    public static final double wheelRadiusMeters =
        Units.inchesToMeters(
            2.19577028); // When using linear characterization: actual linear distance / wheel delta
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
    public static final double driveKp = 0.00021829;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.15812049;
    public static final double driveKv = 0.1165; // From simple characterization: 0.11106210
    public static final double driveKa = 0.029083;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction4i = ((150.0 / 7.0) / (2.0 * Math.PI)); // Mk4i
    public static final double turnMotorReduction4n = ((18.75) / (2.0 * Math.PI));
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
    public static final double pigeonGyroTrimXDegPerRot = 0.0;
    public static final double pigeonGyroTrimYDegPerRot = 0.0;
    public static final double pigeonGyroTrimZDegPerRot = -1.88;

    public static final Angle pigeonMountPoseYaw = Degrees.of(-0.032393235713243484);
    public static final Angle pigeonMountPosePitch = Degrees.of(-0.30914029479026794);
    public static final Angle pigeonMountPoseRoll = Degrees.of(-179.479736328125);

    // Auto config
    public static final boolean mirrorForRedAlliance = true;

    public static final double autoLinearKp = 10.0;
    public static final double autoLinearKi = 0.0;
    public static final double autoLinearKd = 0.0;

    public static final double autoAngularKp = 7.5;
    public static final double autoAngularKi = 0.0;
    public static final double autoAngularKd = 0.0;

    public static final Distance fieldWidth = Meters.of(8.0692);
    public static final Distance fieldLength = Meters.of(16.541);

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

      public static final Pose2d climbSetupPoseOutpost =
          new Pose2d(
              Meters.of(1.7608400583267212), Meters.of(2.085599899291992), Rotation2d.k180deg);
      public static final Pose2d climbPoseOutpost =
          new Pose2d(
              Meters.of(1.7608400583267212), Meters.of(2.085599899291992), Rotation2d.k180deg);
      public static final Pose2d climbSetupPoseDepot =
          new Pose2d(
              Meters.of(1.7608400583267212), Meters.of(2.085599899291992), Rotation2d.k180deg);
      public static final Pose2d climbPoseDepot =
          new Pose2d(
              Meters.of(1.7608400583267212), Meters.of(2.085599899291992), Rotation2d.k180deg);
    }
  }

  public static final class HopperConstants {
    // spindexer constants
    public static final boolean spindexerInverted = false;
    public static final int spindexerCurrentLimit = 20;

    public static final double spindexerKp = 0.0;
    public static final double spindexerKi = 0.0;
    public static final double spindexerKd = 0.0;

    public static final double spindexerKs = 0.0;
    public static final double spindexerKv = 0.0;
    public static final double spindexerKa = 0.0;

    // feeder constants
    public static final boolean feederInverted = false;
    public static final int feederCurrentLimit = 50;

    public static final double feederKp = 2.8084E-08;
    public static final double feederKi = 0.0;
    public static final double feederKd = 0.0;

    public static final double feederKs = 0.12681;
    public static final double feederKv = 0.0017874;
    public static final double feederKa = 0.00010453;
  }

  public static final class IntakeConstants {
    public static final boolean spinnySpinnyInverted = true;
    public static final int spinnySpinnyCurrentLimit = 50;

    public static final double spinnySpinnyKp = 1.5158E-07;
    public static final double spinnySpinnyKi = 0.0;
    public static final double spinnySpinnyKd = 0.0;

    public static final double spinnySpinnyKs = 0.19074;
    public static final double spinnySpinnyKv = 0.0018364;
    public static final double spinnySpinnyKa = 0.00016227;

    public static final boolean uppyDownyInverted = false;
    public static final int uppyDownyCurrentLimit = 30;

    public static final double uppyDownyKp = 0.0;
    public static final double uppyDownyKi = 0.0;
    public static final double uppyDownyKd = 0.0;

    public static final double uppyDownyKs = 0.0;
    public static final double uppyDownyKv = 0.0;
    public static final double uppyDownyKa = 0.0;
  }

  public static final class ShooterConstants {
    public static final Translation2d hubLocation =
        new Translation2d(Inches.of(181.90625), Inches.of(158.84375));
    public static final Translation2d outpostBumpLocation =
        new Translation2d(Inches.of(182.0), Inches.of(90.0));
    public static final Translation2d depotBumpLocation =
        new Translation2d(Inches.of(182.0), fieldWidth.minus(Inches.of(90.0)));

    public static final class FlywheelConstants {
      public static final boolean flywheelInverted = true;
      public static final int flywheelCurrentLimit = 50;

      // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
      public static final double flywheelKp = 6.1453E-07; // From SysId: 0.0000025794
      public static final double flywheelKi = 0.0;
      public static final double flywheelKd = 1.84359E-08;

      public static final double flywheelKs = 0.10996;
      public static final double flywheelKv = 0.0017972; // From SysId: 0.0021194
      public static final double flywheelKa = 0.00028032; // From SysId: 0.0011572
    }

    public static final class HoodConstants {
      public static final boolean hoodInverted = false;
      public static final int hoodCurrentLimit = 50;

      public static final double hoodKp = 20.0;
      public static final double hoodKi = 0.0;
      public static final double hoodKd = 0.0;

      public static final double hoodToZeroKp = 5.0;
      public static final double hoodToZeroKi = 0.0;
      public static final double hoodToZeroKd = 0.0;

      public static final double hoodKs = 0.0;
      public static final double hoodKv = 0.0;
      public static final double hoodKa = 0.0;

      public static final double hoodGearRatio = (17.0 / 20.0) * (20.0 / 340.0);

      public static final Rotation2d hoodMinAngle = Rotation2d.fromDegrees(0.0);
      public static final Rotation2d hoodMaxAngle = Rotation2d.fromDegrees(36.5);
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

      public static final double turretGearRatio = 1.0 / 5.0;

      public static final double turretAbsEncoderGearRatio = (9.0 / 1.0) * (1.0 / 5.0);
      public static final double turretAbsEncoderOffset = 0;
    }
  }

  public static final class VisionConstants {
    public static final Distance maxTagDistance = Meters.of(5.0);

    public static final double maxDistanceStdDev = 999999;
    public static final Angle maxAngleStdDev = Degrees.of(999999);

    public static final boolean useMegaTag2 = true;

    public static record LimelightConstants(String name, Pose3d position) {}
  }
}
