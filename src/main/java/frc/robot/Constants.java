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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.VisionConstants.LimelightConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.QuadranglesUtil;
import frc.robot.util.choreo.ChoreoVars;
import java.util.Arrays;
import java.util.Collections;

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
  public static final double targetHeight = Units.inchesToMeters(120.36); // hub height

  public static enum DriveMode {
    NORMAL,
    DEMO
  };

  public static enum ElasticTab {
    Teleoperated,
    Autonomous,
    Testing
  }

  // #region ROBOT MAP
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
      public static final int kickerCanId = 15;
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
    }

    public static final class VisionConstants {
      public static final LimelightConstants[] aprilTagLimelights = {
        // * Left side
        new LimelightConstants(
            "limelight-square",
            new Pose3d(
                Inches.of(-10.0), // -0.254 m
                Inches.of(-13.239), // -0.3363 m
                Inches.of(8.843), // 0.2246 m
                new Rotation3d(Degrees.of(180), Degrees.of(20), Degrees.of(90)))),
        // * Left back
        new LimelightConstants(
            "limelight-kvale",
            new Pose3d(
                Inches.of(-13.239), // -0.3363 m
                Inches.of(-8.625), // -0.2191 m
                Inches.of(8.843), // 0.2246 m
                new Rotation3d(Degrees.of(180), Degrees.of(20), Degrees.of(180)))),
        // * Right side
        new LimelightConstants(
            "limelight-cube",
            new Pose3d(
                Inches.of(-10.0), // -0.254 m
                Inches.of(13.239), // 0.3363 m
                Inches.of(8.843), // 0.2246 m
                new Rotation3d(Degrees.of(180), Degrees.of(20), Degrees.of(-90)))),
        // * Right back
        new LimelightConstants(
            "limelight-angle",
            new Pose3d(
                Inches.of(-13.239), // -0.3363 m
                Inches.of(8.625), // 0.2191 m
                Inches.of(8.843), // 0.2246 m
                new Rotation3d(Degrees.of(180), Degrees.of(20), Degrees.of(180))))
      };
    }
  }
  // #endregion

  // #region OI
  public static class OIConstants {
    public static final int primaryControllerPort = 0;
    public static final int leftButtonBoardPort = 1;
    public static final int rightButtonBoardPort = 2;

    public static final double controllerStickDeadband = 0.05;
    public static final double controllerTriggerDeadband = 0.25;
    public static final double buttonBoardStickDeadband = 0.05;

    // SHIFT 0 10 Seconds 2:20 – 2:10
    // SHIFT 1 25 Seconds 2:10 - 1:45
    // SHIFT 2 25 Seconds 1:45 - 1:20
    // SHIFT 3 25 Seconds 1:20 - 0:55
    // SHIFT 4 25 Seconds 0:55 - 0:30
    // END GAME 30 Seconds 0:30 - 0:00
    // ! This should not be changed. It is for calculating how much time is left in the given match
    // ! period. To change when controller rumbles, change shiftRumbleTimesSeconds.
    public static final double[] shiftTimesSeconds =
        new double[] {
          130, // 2:10
          105, // 1:45
          80, // 1:20
          55, // 0:55
          30, // 0:30
          0 // 0:00
        };

    // ! Timestamps when the controller STOPS rumbling.
    // (At the shift changes, and in endgame)
    public static long[] shiftRumbleTimesSeconds =
        Arrays.stream(
                new long[] {
                  130, // 2:10
                  105, // 1:45
                  80, // 1:20
                  55, // 0:55
                  30, // 0:30
                  10 // 0:10
                })
            // ! MUST be descending order for OI.shiftRumbleWindow() to work
            .boxed()
            .sorted(Collections.reverseOrder())
            .mapToLong(Long::longValue)
            .toArray();

    public static boolean shiftRumbleEnabled = true;
    public static double shiftRumbleIntensity = 0.50;
    // Since match time FLOORS instead of ROUNDS, offset by 1s to make rumble seem like it starts at
    // the right time
    public static double shiftRumbleOffsetSeconds = 1.0;
    public static double shiftRumbleContinuousSeconds = 2.0;
    public static double shiftRumblePulseOnSeconds = 0.25;
    public static double shiftRumblePulseOffSeconds = 0.75;
    public static int shiftRumblePulseCount = 3;
  }
  // #endregion

  // #region CLIMBER
  public static class ClimberConstants {
    public static final boolean climberInverted = true;
    public static Current climberCurrentLimit = Amps.of(70);
    public static Time climberRampRate = Milliseconds.of(10);
    public static final double climberGearRatio = (1.0 / 5.0) * (1.0 / 9.0);

    public static final int climberCurrentSensingFilterSize = 10;

    public static double climberUpPosition = 3.067;
    public static double climberDownPosition = 0.0;
    // Climber should go 80% of the way DOWN when robot is climbing
    public static double climberClimbPosition = Climber.percentClimbedPosition(0.8);

    public static double climberTolerance = 0.01;

    public static double climberKp = 10.0;
    public static double climberKi = 0.0;
    public static double climberKd = 0.0;

    public static double climberKs = 0.0;
    public static double climberKv = 0.0;
    public static double climberKa = 0.0;
  }
  // #endregion

  // #region DRIVE
  public static class DriveConstants {
    /*
    ! Things that need to be configured in addition to AdvantageKit Swerve Template configs:

    * maxAngularSpeedFactor
       - Units: rad/sec
       - Divide max rotation speed when driving by max rotation speed while stationary
    */

    public static double maxSpeedMetersPerSec = 4.62906; // 15.187 ft/s
    public static double maxShootingSpeedMetersPerSec = Units.feetToMeters(4.0);
    // * Max rotation speed (Rad/Sec) while moving / Max rotation speed while stationary
    public static double maxAngularSpeedRadPerSec = Units.degreesToRadians(360 + 72);
    public static double maxShootingAngularSpeedRadPerSec = Units.degreesToRadians(110);
    public static final double demoModeSpeedFactor = 0.15;

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
    // ! Gears on the right
    public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromRadians(1.226);
    public static final Rotation2d frontRightZeroRotation = Rotation2d.fromRadians(3.369);
    public static final Rotation2d backLeftZeroRotation = Rotation2d.fromRadians(3.57);
    public static final Rotation2d backRightZeroRotation = Rotation2d.fromRadians(5.12);

    // Drive motor configuration
    public static final boolean[] driveInverted = new boolean[] {true, true, true, true};
    public static final int driveMotorCurrentLimit = 60;
    public static final Time driveRampRate = Milliseconds.of(10);
    // When using linear characterization: actual linear distance / wheel delta
    public static final double wheelRadiusMeters =
        Units.inchesToMeters(2.00302745); // From rotational characterization
    public static final double driveMotorReduction =
        (50.0 / 16.0) * (19.0 / 25.0) * (45.0 / 15.0); // SDS Mk4n/4i L1+ Gearing
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
    // Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
    // Wheel Rad/Sec

    // Drive PID configuration - From SysId
    public static double driveKp = 7.43E-06;
    public static double driveKi = 0.0;
    public static double driveKd = 0.0;
    public static double driveKs = 0.11;
    public static double driveKv = 0.1179175;
    public static double driveKa = 0.02385725;
    public static final double driveSimP = 0.05;
    public static final double driveSimI = 0.0;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;
    public static final double driveSimKa = 0.0;

    // Turn motor configuration
    public static final boolean[] turnInverted = new boolean[] {true, true, true, true};
    public static final int[] turnMotorCurrentLimit = new int[] {40, 40, 40, 40};
    public static final Time turnRampRate = Milliseconds.of(10);
    public static final double[] turnMotorReduction =
        new double[] {
          ((18.75) / (2.0 * Math.PI)), // Mk4i w/ 16t adapter
          ((18.75) / (2.0 * Math.PI)), // Mk4i w/ 16t adapter
          ((18.75) / (2.0 * Math.PI)), // Mk4n
          ((18.75) / (2.0 * Math.PI)) // Mk4n
        };
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean[] turnAbsEncoderInverted =
        new boolean[] {false, false, false, false};
    public static final boolean[] turnRelEncoderInverted = new boolean[] {true, true, true, true};
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static double turnKp = 2.0;
    public static double turnKi = 0.0;
    public static double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimI = 0.0;
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
    public static final double pigeonGyroTrimZDegPerRot = -3.543375;

    public static final Angle pigeonMountPoseYaw = Degrees.of(-0.03239322453737259);
    public static final Angle pigeonMountPosePitch = Degrees.of(-0.30914023518562317);
    public static final Angle pigeonMountPoseRoll = Degrees.of(-179.48013305664062);

    // Auto config
    public static final boolean mirrorForRedAlliance = true;

    public static double autoLinearKp = 10.0;
    public static double autoLinearKi = 0.0;
    public static double autoLinearKd = 0.0;

    public static double autoAngularKp = 7.5;
    public static double autoAngularKi = 0.0;
    public static double autoAngularKd = 0.0;

    public static final Distance fieldWidth = ChoreoVars.FieldWidth;
    public static final Distance fieldLength = ChoreoVars.FieldLength;

    public static final Translation2d fieldSize = new Translation2d(fieldLength, fieldWidth);

    // #region AUTOALIGN
    public static class AutoAlignConstants {
      // Tuned with Ziegler-Nichols for classic PID
      // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
      // kU is 0.5, tU is 0.193s (9.65 robot loops)
      public static double autoAlignLinearKp = 4.0;
      public static double autoAlignLinearKi = 0;
      public static double autoAlignLinearKd = 0.1;
      public static Distance autoAlignLinearTolerance = Centimeters.of(1.0);
      public static Rotation2d autoAlignAngularTolerance = Rotation2d.fromDegrees(1.0);

      public static double autoAlignAngularKp = 5.0;
      public static double autoAlignAngularKi = 0.0;
      public static double autoAlignAngularKd = 0.1;

      public static final Pose2d climbSetupPoseOutpost = ChoreoVars.Poses.ClimbSetupOutpost;
      public static final Pose2d climbPoseOutpost = ChoreoVars.Poses.ClimbOutpost;
      public static final Pose2d climbSetupPoseDepot = ChoreoVars.Poses.ClimbSetupDepot;
      public static final Pose2d climbPoseDepot = ChoreoVars.Poses.ClimbDepot;

      public static Distance trenchXTolerance = fieldLength;
      public static Distance trenchYTolerance = Inches.of(5.0);
      public static Rotation2d trenchAngularTolerance = Rotation2d.fromDegrees(2.0);

      // X value beyond which we align to the opposite alliance's trenches
      public static Distance closerToOppositeTrenchLine = fieldLength.div(2.0).plus(Feet.of(9.0));
      public static Distance preTrenchOffset = Feet.of(4);
      public static Distance postTrenchOffset = Feet.of(8);

      public static final Translation2d closeLeftTrench =
          ChoreoVars.Poses.LeftTrench.getTranslation();
      public static final Translation2d closeRightTrench =
          ChoreoVars.Poses.RightTrench.getTranslation();
      public static final Translation2d farLeftTrench =
          QuadranglesUtil.flipTranslation(ChoreoVars.Poses.LeftTrench.getTranslation());
      public static final Translation2d farRightTrench =
          QuadranglesUtil.flipTranslation(ChoreoVars.Poses.RightTrench.getTranslation());
    }
  }
  // #endregion
  // #endregion

  // #region HOPPER
  public static final class HopperConstants {
    // Spindexer constants
    public static final boolean spindexerInverted = true;
    public static Current spindexerCurrentLimit = Amps.of(30);
    public static Time spindexerRampRate = Milliseconds.of(0);
    public static final double spindexerGearRatio = 180.0 / 6293.0;

    public static final int spindexerCurrentSensingFilterSize = 10;

    public static AngularVelocity spindexerSpeed = RPM.of(90);
    public static AngularVelocity spindexerIntakingSpeed = RPM.of(10);

    public static double spindexerKp = 2.342E-07;
    public static double spindexerKi = 0.0;
    public static double spindexerKd = 0.0;

    public static double spindexerIMaxAccum = 1.0;
    public static double spindexerIZone = 50;

    public static double spindexerKs = 0.086873;
    public static double spindexerKv = 0.063322;
    public static double spindexerKa = 0.0028565;

    // Kicker constants
    public static final boolean kickerInverted = false;
    public static final int kickerCurrentLimit = 50;
    public static final Time kickerRampRate = Milliseconds.of(0);

    public static final AngularVelocity kickerMaxSpeed = RPM.of(4000);
    // Number to multiply the flywheel speed by
    // public static double kickerSpeedFactor = 1.0;
    public static AngularVelocity kickerSpeed = RPM.of(4000);

    public static double kickerKp = 6.9449E-08;
    public static double kickerKi = 0.0;
    public static double kickerKd = 0.0;

    public static double kickerKs = 0.11154;
    public static double kickerKv = 0.0017758;
    public static double kickerKa = 0.00012735;
  }
  // #endregion

  // #region INTAKE
  public static final class IntakeConstants {
    public static final boolean spinnySpinnyInverted = true;
    public static final Current spinnySpinnyCurrentLimit = Amps.of(50);
    public static final Time spinnySpinnyRampRate = Milliseconds.of(0);
    public static final double spinnySpinnyGearRatio = 17.0 / 55.0;

    public static AngularVelocity intakeSpinnySpinnySpeed = RPM.of(2000);
    public static AngularVelocity intakeSpinnySpinnyShootingSpeed = RPM.of(1000);

    public static double spinnySpinnyKp = 3.1048E-05;
    public static double spinnySpinnyKi = 0.0;
    public static double spinnySpinnyKd = 0.0;

    public static double spinnySpinnyKs = 0.099025;
    public static double spinnySpinnyKv = 0.0058905;
    public static double spinnySpinnyKa = 0.00033752;

    public static final boolean uppyDownyInverted = false;
    public static final int uppyDownyCurrentLimit = 50;
    public static final Time uppyDownyRampRate = Milliseconds.of(10);
    public static final double uppyDownyGearRatio = 1.0 / 4.0;
    public static final int uppyDownCurrentSensingFilterSize = 10;

    public static final double uppyDownyMinPosition = 0.0;
    public static final double uppyDownyMaxPosition = 0.0;

    public static double uppyDownyKp = 0.01;
    public static double uppyDownyKi = 0.0;
    public static double uppyDownyKd = 0.0;

    public static double uppyDownyKs = 0.0;
    public static double uppyDownyKv = 0.007;
    public static double uppyDownyKa = 0.0;

    public static double uppyDownyRaiseRPM = -800.0;
    public static double uppyDownyLowerRPM = 150.0;

    public static double jostleIntakeUpTime = 0.9;
    public static double jostleIntakeDownTime = 0.7;
  }
  // #endregion

  // #region SHOOTER
  public static final class ShooterConstants {
    public static final Translation2d hubLocation = ChoreoVars.Poses.Hub.getTranslation();
    public static final Translation2d nzDepotShootingTarget =
        ChoreoVars.Poses.NZDepotShootingTarget.getTranslation();
    public static final Translation2d nzOutpostShootingTarget =
        ChoreoVars.Poses.NZOutpostShootingTarget.getTranslation();

    // The distance into the NZ beyond which shooter targets middle of AZ rather than Hub
    // 17.625 inches is robot center-to-bumper edge distance
    public static final Distance azLineOffset = Inches.of(17.625).plus(Inches.of(48));
    // TODO: Move to a more appropriate place
    public static final Distance azLine = Inches.of(158.6).plus(azLineOffset);

    public static final double gravity = 9.81;

    public static final double shooterX = 0.0;
    public static final double shooterY = Units.inchesToMeters(-2.074);
    public static final double shooterZ = Units.inchesToMeters(13.72);

    // #region LINEAR INTERP
    public static class AimShooterMathLinearConstants {
      public static double robotYawKv = 0.08;

      public static final double turretTrimDefaultRot = Units.degreesToRotations(0.0);
      public static double turretTrimRot = turretTrimDefaultRot;
      public static final double turretTrimIncrementRot = Units.degreesToRotations(1.0);

      public static Rotation2d azHoodTrim = Rotation2d.kZero;
      public static Rotation2d nzHoodTrim = Rotation2d.kZero;
      public static AngularVelocity azFlywheelTrim = RPM.zero();
      public static AngularVelocity nzFlywheelTrim = RPM.zero();

      public static final Distance distanceTrimIncrement = Inches.of(5.0);
      public static final Distance xyTrimIncrement = Inches.of(5.0);

      public static Distance azDistanceTrimDefault = Inches.of(40.0);
      public static Distance azDistanceTrim = azDistanceTrimDefault;
      public static Distance azXTrim = Inches.zero();
      public static Distance azYTrim = Inches.zero();

      public static Distance nzDistanceTrimDefault = Inches.of(30.0);
      public static Distance nzDistanceTrim = nzDistanceTrimDefault;
      public static Distance nzXTrim = Inches.zero();
      public static Distance nzYTrim = Inches.zero();

      // ! Spreadsheet with data points & extrapolation work
      // https://docs.google.com/spreadsheets/d/1F3wifhI_nVvJh-EjeXkzgORYJy9NPVHc9NdVFYBNev0/edit?usp=sharing
      public static Time azTOFAdjustment = Seconds.of(0.0);
      public static final LinearInterpolationDataPoint[] azLinearInterpolationDataPoints =
          new LinearInterpolationDataPoint[] {
            // * Tuned side towards hub
            new LinearInterpolationDataPoint(
                Meters.of(2.09),
                Rotation2d.fromDegrees(24.2238027),
                RPM.of(2800),
                Seconds.of(1.033)),
            // * Tuned back towards hub
            new LinearInterpolationDataPoint(
                Meters.of(2.85), Rotation2d.fromDegrees(30), RPM.of(3000), Seconds.of(1.034)),
            new LinearInterpolationDataPoint(
                Meters.of(3.27), Rotation2d.fromDegrees(32), RPM.of(3100), Seconds.of(1.033)),
            new LinearInterpolationDataPoint(
                Meters.of(3.53), Rotation2d.fromDegrees(34), RPM.of(3150), Seconds.of(1.033)),
            new LinearInterpolationDataPoint(
                Meters.of(3.77), Rotation2d.fromDegrees(35), RPM.of(3200), Seconds.of(1.066)),
            new LinearInterpolationDataPoint(
                Meters.of(3.98), Rotation2d.fromDegrees(37), RPM.of(3300), Seconds.of(1.1)),
            new LinearInterpolationDataPoint(
                Meters.of(4.2), Rotation2d.fromDegrees(37), RPM.of(3325), Seconds.of(1.099)),
            new LinearInterpolationDataPoint(
                Meters.of(4.4), Rotation2d.fromDegrees(36), RPM.of(3400), Seconds.of(1.167)),
            new LinearInterpolationDataPoint(
                Meters.of(4.71), Rotation2d.fromDegrees(37.5), RPM.of(3500), Seconds.of(1.166)),
            new LinearInterpolationDataPoint(
                Meters.of(4.92), Rotation2d.fromDegrees(38), RPM.of(3550), Seconds.of(1.197)),
            new LinearInterpolationDataPoint(
                Meters.of(5.26), Rotation2d.fromDegrees(39), RPM.of(3600), Seconds.of(1.216)),
            // ! Last points extrapolated
            new LinearInterpolationDataPoint(
                Meters.of(6.29545454545455),
                Rotation2d.fromDegrees(45),
                RPM.of(3890.11363636364),
                Seconds.of(1.26786818181818)),
            new LinearInterpolationDataPoint(
                Meters.of(14.3793103448276),
                Rotation2d.fromDegrees(45),
                RPM.of(6000),
                Seconds.of(1.82242068965517)),
          };

      public static Time nzTOFAdjustment = Seconds.of(0.0);
      public static final LinearInterpolationDataPoint[] nzLinearInterpolationDataPoints =
          new LinearInterpolationDataPoint[] {
            new LinearInterpolationDataPoint(
                Meters.of(7.270), Rotation2d.fromDegrees(45), RPM.of(3900.0), Seconds.of(1.425)),
            new LinearInterpolationDataPoint(
                Meters.of(5.650), Rotation2d.fromDegrees(45), RPM.of(3300.0), Seconds.of(1.330)),
            new LinearInterpolationDataPoint(
                Meters.of(4.410), Rotation2d.fromDegrees(45), RPM.of(2800.0), Seconds.of(1.150)),
            new LinearInterpolationDataPoint(
                Meters.of(3.220), Rotation2d.fromDegrees(45), RPM.of(2400.0), Seconds.of(1.0)),
          };

      public static record LinearInterpolationDataPoint(
          Distance distance,
          Rotation2d hoodAngle,
          AngularVelocity flywheelSpeed,
          Time timeOfFlight) {}
    }
    // #endregion

    // #region FLYWHEEL
    public static final class FlywheelConstants {
      public static final boolean flywheelInverted = true;
      public static final int flywheelCurrentLimit = 50;
      public static final Time flywheelRampRate = Milliseconds.of(0);
      public static final double flywheelRadius = Units.inchesToMeters(4) / 2.0;

      public static final AngularVelocity flywheelMinSpeed = RPM.of(0.0);
      public static final AngularVelocity flywheelMaxSpeed = RPM.of(6000.0);

      // Fraction of target speed at which kicker triggers and robot starts shooting
      public static double flywheelThresholdFactor = 0.99;
      // Default starting speed for shots using dashboard settings
      public static AngularVelocity flywheelManualSpeed = RPM.of(3000);

      // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
      public static double flywheelKp = (3.5004E-07 + 2.7377E-07) / 2.0;
      public static double flywheelKi = 0.0;
      public static double flywheelKd = 0.0;

      public static double flywheelMaxIAccum = 10000;
      public static double flywheelIZone = 1000;

      public static double flywheelKs = (0.11624 + 0.12047) / 2.0;
      public static double flywheelKv = (0.0017858 + 0.0018076) / 2.0;
      public static double flywheelKa = (0.00021404 + 0.00017531) / 2.0;
    }
    // #endregion

    // #region HOOD
    public static final class HoodConstants {
      public static final boolean hoodInverted = false;
      public static final int hoodCurrentLimit = 50;
      public static final Time hoodRampRate = Milliseconds.of(0);
      public static final double hoodGearRatio = (17.0 / 20.0) * (20.0 / 340.0);
      public static final int hoodCurrentSensingFilterSize = 10;

      public static final Rotation2d hoodMinAngle = Rotation2d.fromDegrees(24.2238027);
      public static final Rotation2d hoodMaxAngle = Rotation2d.fromDegrees(45.0);

      // Default hood angle for shots using dashboard settings
      public static Rotation2d hoodManualAngle = hoodMinAngle;
      // Speed of manually moving hood up and down
      public static Rotation2d hoodManualIncrement = Rotation2d.fromDegrees(2.0);

      public static double hoodKp = 20.0;
      public static double hoodKi = 0.0;
      public static double hoodKd = 0.0;

      public static double hoodToZeroKp = 5.0;
      public static double hoodToZeroKi = 0.0;
      public static double hoodToZeroKd = 0.0;

      public static double hoodKs = 0.0;
      public static double hoodKv = 0.0;
      public static double hoodKa = 0.0;

      public static final double hoodRezeroTimeoutSeconds = 10.0;

      public static double trenchSafetyBufferMeters = 0.5;
      public static double trenchSafetyLookaheadSeconds = 0.5;
    }
    // #endregion

    // #region TURRET
    public static final class TurretConstants {
      public static final boolean turretInverted = true;
      public static final int turretCurrentLimit = 50;
      public static final Time turretRampRate = Milliseconds.of(0);

      public static final double turretMinAngleRot = Units.degreesToRotations(-45);
      public static final double turretMaxAngleRot = Units.degreesToRotations(360);

      // Turret shooting over the back of the robot
      public static double turretRezeroLocationRot = Units.degreesToRotations(180);
      public static double turretShootingToleranceRot = Units.degreesToRotations(20.0);

      // Speed of manually moving turret CW/CCW
      public static double turretManualIncrementRot = Units.degreesToRotations(2.0);

      public static double turretPositionToleranceRot = Units.degreesToRotations(0.1);
      // Retractor kicks in when CW from (less than) this position
      public static double turretCableRetractorStartRot = Units.degreesToRotations(115);
      // Feedforward for retractor when turret is moving clockwise (against the retractor)
      public static Voltage turretCableRetractorFFCW = Volts.of(0.0); // -0.3
      // Feedforward for retractor when turret is moving counterclockwise (with the retractor)
      public static Voltage turretCableRetractorFFCCW = Volts.of(0.0); // -0.1

      public static double turretKp = 3.0;
      public static double turretKi = 0.0;
      public static double turretKd = 0.0;

      public static double turretIMaxAccumRot = Units.degreesToRotations(10) * 1000;
      public static double turretIZoneRot = Units.degreesToRotations(20);

      public static double turretKs = 0.16;
      public static double turretKv = 0.0;
      public static double turretKa = 0.0;

      public static final int turretSetpointFilterSize = 1;

      public static final double turretGearRatio = (17.0 / 115.0) * (1.0 / 3.0);
    }
    // #endregion
  }
  // #endregion

  // #region VISION
  public static final class VisionConstants {
    public static final boolean useMegaTag2 = true;

    public static final Distance maxTagDistanceMT1 = Meters.of(5.0);
    public static final Distance maxTagDistanceMT2 = fieldWidth;
    public static final int minTagCountMT1 = 2;
    public static final int minTagCountMT2 = 1;

    public static double distanceStdDev = 1.2;
    public static double angleStdDev = 999999;

    // * AprilTag Locations:
    // *
    // https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf#%5B%7B%22num%22%3A78%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C33%2C565%2C0%5D
    public static final int[] enabledAprilTags =
        new int[] {
          1, // Red Outpost Trench, Facing NZ
          2, // Red Hub, Facing Outpost Side, Nearer to NZ
          3, // Red Hub, Facing NZ, Nearer to Outpost
          4, // Red Hub, Facing NZ, Nearer to Depot
          5, // Red Hub, Facing Depot Side, Nearer to NZ
          6, // Red Depot Trench, Facing NZ
          7, // Red Depot Trench, Facing AZ
          8, // Red Hub, Facing Depot Side, Nearer to AZ
          9, // Red Hub, Facing Tower, Nearer to Depot
          10, // Red Hub, Facing Tower, Nearer to Outpost
          11, // Red Hub, Facing Outpost Side, Nearer to AZ
          12, // Red Outpost Trench, Facing AZ
          13, // Red Outpost, Nearer to Human Player
          14, // Red Outpost, Nearer to Tower
          15, // Red Tower, Nearer to Outpost
          16, // Red Tower, Nearer to Depot
          17, // Blue Outpost Trench, Facing NZ
          18, // Blue Hub, Facing Outpost Side, Nearer to NZ
          19, // Blue Hub, Facing NZ, Nearer to Outpost
          20, // Blue Hub, Facing NZ, Nearer to Depot
          21, // Blue Hub, Facing Depot Side, Nearer to NZ
          22, // Blue Depot Trench, Facing NZ
          23, // Blue Depot Trench, Facing AZ
          24, // Blue Hub, Facing Depot Side, Nearer to AZ
          25, // Blue Hub, Facing Tower, Nearer to Depot
          26, // Blue Hub, Facing Tower, Nearer to Outpost
          27, // Blue Hub, Facing Tower, Nearer to Depot
          28, // Blue Outpost Trench, Facing AZ
          29, // Blue Outpost, Nearer to Human Player
          30, // Blue Outpost, Nearer to Tower
          31, // Blue Tower, Nearer to Outpost
          32 // Blue Tower, Nearer to Depot
        };

    public static record LimelightConstants(String name, Pose3d position) {}
  }
  // #endregion
}
