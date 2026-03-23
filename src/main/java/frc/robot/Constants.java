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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.VisionConstants.LimelightConstants;
import frc.robot.util.choreo.ChoreoVars;

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
    public static final int climberCurrentLimit = 50; // !  Was 70
    // 1:80 ratio is to prevent encoder from looping
    public static final double climberGearRatio = (1.0 / 5.0) * (1.0 / 9.0);

    public static final int climberCurrentSensingFilterSize = 10;

    public static final double climberDownPosition = 2.443155;
    public static final double climberUpPosition = 0.0;

    public static double climberKp = 10.0;
    public static double climberKi = 0.0;
    public static double climberKd = 0.0;

    public static double climberKs = 0.0;
    public static double climberKv = 0.0;
    public static double climberKa = 0.0;
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
    // ! Gears on the right
    public static final Rotation2d frontLeftZeroRotation = Rotation2d.fromRadians(1.795);
    public static final Rotation2d frontRightZeroRotation = Rotation2d.fromRadians(2.824);
    public static final Rotation2d backLeftZeroRotation = Rotation2d.fromRadians(1.407);
    public static final Rotation2d backRightZeroRotation = Rotation2d.fromRadians(4.0498);

    // Drive motor configuration
    public static final boolean[] driveInverted = new boolean[] {true, true, true, true};
    public static final int driveMotorCurrentLimit = 50;
    public static final double wheelRadiusMeters =
        Units.inchesToMeters(
            2.29793665); // When using linear characterization: actual linear distance / wheel delta
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
    public static double driveKp = 2.95E-05; // 0.00021829
    public static double driveKi = 0.0;
    public static double driveKd = 0.0;
    public static double driveKs = 0.1619433333; // 0.15812049
    public static double driveKv = 0.1124575; // 0.1165 // From simple characterization: 0.11106210
    public static double driveKa = 0.02673925; // 0.029083
    public static double driveSimP = 0.05;
    public static double driveSimI = 0.0;
    public static double driveSimD = 0.0;
    public static double driveSimKs = 0.0;
    public static double driveSimKv = 0.0789;
    public static double driveSimKa = 0.0;

    // Turn motor configuration
    public static final boolean[] turnInverted = new boolean[] {true, true, true, true};
    public static final int[] turnMotorCurrentLimit = new int[] {25, 25, 20, 20};
    public static final double[] turnMotorReduction =
        new double[] {
          ((150.0 / 7.0) / (2.0 * Math.PI)), // Mk4i
          ((150.0 / 7.0) / (2.0 * Math.PI)), // Mk4i
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
    public static double turnSimP = 8.0;
    public static double turnSimI = 0.0;
    public static double turnSimD = 0.0;
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

    public static class AutoAlignConstants {
      // Tuned with Ziegler-Nichols for classic PID
      // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
      // kU is 0.5, tU is 0.193s (9.65 robot loops)
      public static double autoAlignLinearKp = 4.0;
      public static double autoAlignLinearKi = 0;
      public static double autoAlignLinearKd = 0.1;
      public static final Distance autoAlignLinearTolerance = Centimeters.of(1.0);
      public static final Rotation2d autoAlignAngularTolerance = Rotation2d.fromDegrees(1.0);

      public static double autoAlignAngularKp = 5.0;
      public static double autoAlignAngularKi = 0.0;
      public static double autoAlignAngularKd = 0.1;

      public static final Pose2d climbSetupPoseOutpost = ChoreoVars.Poses.ClimbSetupOutpost;
      public static final Pose2d climbPoseOutpost = ChoreoVars.Poses.ClimbOutpost;
      public static final Pose2d climbSetupPoseDepot = ChoreoVars.Poses.ClimbSetupDepot;
      public static final Pose2d climbPoseDepot = ChoreoVars.Poses.ClimbDepot;
    }
  }

  public static final class HopperConstants {
    // spindexer constants
    public static final boolean spindexerInverted = true;
    public static final int spindexerCurrentLimit = 20;
    public static final double spindexerGearRatio = 180.0 / 6293.0;

    public static double spindexerKp = 0.01; // 5.2634E-05
    public static double spindexerKi = 1E-06;
    public static double spindexerKd = 0.0;

    public static double spindexerIMaxAccum = 1.0;
    public static double spindexerIZone = 50;

    public static double spindexerKs = 0.019266;
    public static double spindexerKv = 0.065; // 0.066119
    public static double spindexerKa = 0.010955; // 0.019434

    // kicker constants
    public static final boolean kickerInverted = false;
    public static final int kickerCurrentLimit = 50;

    public static double kickerKp = 2.8084E-08;
    public static double kickerKi = 0.0;
    public static double kickerKd = 0.0;

    public static double kickerKs = 0.12681;
    public static double kickerKv = 0.0017874;
    public static double kickerKa = 0.00010453;
  }

  public static final class IntakeConstants {
    public static final boolean spinnySpinnyInverted = true;
    public static final int spinnySpinnyCurrentLimit = 50;
    public static final double spinnySpinnyGearRatio = 17.0 / 55.0;

    public static double spinnySpinnyKp = 3.1048E-05;
    public static double spinnySpinnyKi = 0.0;
    public static double spinnySpinnyKd = 0.0;

    public static double spinnySpinnyKs = 0.099025;
    public static double spinnySpinnyKv = 0.0058905;
    public static double spinnySpinnyKa = 0.00033752;

    public static final boolean uppyDownyInverted = false;
    public static final int uppyDownyCurrentLimit = 50;
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

    public static final double uppyDownyRaiseRPM = -500.0;
    public static final double uppyDownyLowerRPM = 150.0;

    public static final double jostleIntakeUpTime = 0.75;
    public static final double jostleIntakeDownTime = 0.5;
  }

  public static final class ShooterConstants {
    public static final Translation2d hubLocation = ChoreoVars.Poses.Hub.getTranslation();
    public static final Translation2d nzDepotShootingTarget =
        ChoreoVars.Poses.NZDepotShootingTarget.getTranslation();
    public static final Translation2d nzOutpostShootingTarget =
        ChoreoVars.Poses.NZOutpostShootingTarget.getTranslation();

    // The distance into the NZ beyond which shooter targets middle of AZ rather than Hub
    public static final Distance azLineOffset = Inches.of(12);
    // TODO: Move to a more appropriate place
    public static final Distance azLine = Inches.of(182.11).plus(Inches.of(12));

    public static final double gravity = 9.81;

    public static final double shooterX = 0.0;
    public static final double shooterY = Units.inchesToMeters(-2.074);
    public static final double shooterZ = Units.inchesToMeters(13.72);

    public static final LinearInterpolationDataPoint[] linearInterpolationDataPoints =
        new LinearInterpolationDataPoint[] {
          // TODO: need NZ shot time of flight
          new LinearInterpolationDataPoint(
              Meters.of(9.170), Rotation2d.fromDegrees(45.0), RPM.of(4500.0), Seconds.of(0.0)),
          new LinearInterpolationDataPoint(
              Meters.of(5.530), Rotation2d.fromDegrees(45.0), RPM.of(3750.0), Seconds.of(1.26)),
          new LinearInterpolationDataPoint(
              Meters.of(3.315), Rotation2d.fromDegrees(35.0), RPM.of(3200.0), Seconds.of(1.25)),
          new LinearInterpolationDataPoint(
              Meters.of(2.157), Rotation2d.fromDegrees(30.0), RPM.of(3000.0), Seconds.of(1.2)),
          new LinearInterpolationDataPoint(
              Meters.of(1.492),
              Rotation2d.fromDegrees(24.2238027),
              RPM.of(2800.0),
              Seconds.of(1.16)),
          new LinearInterpolationDataPoint(
              Meters.of(1.053),
              Rotation2d.fromDegrees(24.2238027),
              RPM.of(2800.0),
              Seconds.of(1.16))
        };

    public static record LinearInterpolationDataPoint(
        Distance distance,
        Rotation2d hoodAngle,
        AngularVelocity flywheelSpeed,
        Time timeOfFlight) {}

    public static final class FlywheelConstants {
      public static final boolean flywheelInverted = true;
      public static final int flywheelCurrentLimit = 50;
      public static final double flywheelRadius = Units.inchesToMeters(4) / 2.0;

      public static final AngularVelocity flywheelMinSpeed = RPM.of(0.0);
      public static final AngularVelocity flywheelMaxSpeed = RPM.of(5700.0);

      // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
      public static double flywheelKp = 6.1453E-07; // From SysId: 0.0000025794
      public static double flywheelKi = 1E-07;
      public static double flywheelKd = 1.84359E-08;

      public static double flywheelMaxIAccum = 10000;
      public static double flywheelIZone = 1000;

      public static double flywheelKs = 0.10996;
      public static double flywheelKv = 0.0017972; // From SysId: 0.0021194
      public static double flywheelKa = 0.00028032; // From SysId: 0.0011572
    }

    public static final class HoodConstants {
      public static final boolean hoodInverted = false;
      public static final int hoodCurrentLimit = 50;
      public static final double hoodGearRatio = (17.0 / 20.0) * (20.0 / 340.0);
      public static final int hoodCurrentSensingFilterSize = 10;

      public static double hoodKp = 20.0;
      public static double hoodKi = 0.0;
      public static double hoodKd = 0.0;

      public static double hoodToZeroKp = 5.0;
      public static double hoodToZeroKi = 0.0;
      public static double hoodToZeroKd = 0.0;

      public static double hoodKs = 0.0;
      public static double hoodKv = 0.0;
      public static double hoodKa = 0.0;

      public static final Rotation2d hoodMinAngle = Rotation2d.fromDegrees(24.2238027);
      public static final Rotation2d hoodMaxAngle = Rotation2d.fromDegrees(45.0);

      public static final double hoodRezeroTimeoutSeconds = 10.0;
    }

    public static final class TurretConstants {
      public static final boolean turretInverted = true;
      public static final int turretCurrentLimit = 50;

      public static final double turretMinAngleRot = Units.degreesToRotations(-45);
      public static final double turretMaxAngleRot = Units.degreesToRotations(360);

      public static double turretPositionTolerance = Units.degreesToRotations(0.4);
      // Retractor kicks in when CW from (less than) this position
      public static final double turretCableRetractorStart = Units.degreesToRotations(115);
      public static final Voltage turretCableRetractorFFCW = Volts.of(-0.3);
      public static final Voltage turretCableRetractorFFCCW = Volts.of(-0.1);

      public static final double turretYawVelocityFactor = 0.5;

      public static double turretKp = 0.55;
      public static double turretKi = 0.002;
      public static double turretKd = 10;

      public static double turretIMaxAccum = Units.degreesToRotations(10) * 1000;
      public static double turretIZone = Units.degreesToRotations(20);

      public static double turretKs = 0.28642;
      public static double turretKv = 0.1;
      public static double turretKa = 0.013128;

      public static final int turretSetpointFilterSize = 30;

      public static final double turretGearRatio = (17.0 / 115.0) * (1.0 / 3.0);

      public static final double turretAbsEncoderGearRatio =
          (17.0 / 115.0) * (60.0 / 20.0) * (60.0 / 20.0);

      public static final double turretAbsEncoderOffset =
          0.59 - Units.degreesToRotations(90) / turretAbsEncoderGearRatio;
    }
  }

  public static final class VisionConstants {
    public static final boolean useMegaTag2 = true;

    public static final Distance maxTagDistance = Meters.of(5.0);
    public static final int minTagCountMT1 = 2;
    public static final int minTagCountMT2 = 1;

    public static final double maxDistanceStdDev = 1.2;
    public static final double maxAngleStdDev = 999999;

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
}
