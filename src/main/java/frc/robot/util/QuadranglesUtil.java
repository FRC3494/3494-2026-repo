package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public final class QuadranglesUtil {
  // #region MATH
  public static Pose2d flipPose(Pose2d pose) {
    return new Pose2d(flipTranslation(pose.getTranslation()), flipAngle(pose.getRotation()));
  }

  public static Pose2d toAlliancePose(Pose2d bluePose) {
    if (alliance == Alliance.Blue) {
      return bluePose;
    } else {
      return flipPose(bluePose);
    }
  }

  public static Translation2d flipTranslation(Translation2d translation) {
    return fieldSize.minus(translation);
  }

  public static Translation2d toAllianceTranslation(Translation2d blueTranslation) {
    if (alliance == Alliance.Blue) {
      return blueTranslation;
    } else {
      return flipTranslation(blueTranslation);
    }
  }

  public static Rotation2d flipAngle(Rotation2d angle) {
    return angle.rotateBy(Rotation2d.k180deg);
  }

  public static Rotation2d toAllianceAngle(Rotation2d blueAngle) {
    if (alliance == Alliance.Blue) {
      return blueAngle;
    } else {
      return flipAngle(blueAngle);
    }
  }

  public static Distance toAllianceX(Distance blueX) {
    if (alliance == Alliance.Blue) {
      return blueX;
    } else {
      return fieldLength.minus(blueX);
    }
  }

  public static double toAllianceXMeters(double blueXMeters) {
    if (alliance == Alliance.Blue) {
      return blueXMeters;
    } else {
      return fieldLength.in(Meters) - blueXMeters;
    }
  }

  public static Distance toAllianceY(Distance blueY) {
    if (alliance == Alliance.Blue) {
      return blueY;
    } else {
      return fieldWidth.minus(blueY);
    }
  }

  public static double toAllianceYMeters(double blueYMeters) {
    if (alliance == Alliance.Blue) {
      return blueYMeters;
    } else {
      return fieldWidth.in(Meters) - blueYMeters;
    }
  }

  /** Returns `true` if `inputTranslation` is closer to `a`, `false` if closer to `b`. */
  public static boolean closerTo(Translation2d a, Translation2d b, Translation2d inputTranslation) {
    double distanceToA = inputTranslation.getDistance(a);
    double distanceToB = inputTranslation.getDistance(b);

    return distanceToA <= distanceToB;
  }

  /** Same as `closerTo()` but `a` and `b` are both blue-side poses. */
  public static boolean closerToWithFlip(
      Translation2d a, Translation2d b, Translation2d inputTranslation) {
    return closerTo(a, b, toAllianceTranslation(inputTranslation));
  }

  public static boolean closerTo(double a, double b, double input) {
    double distanceToA = Math.abs(a - input);
    double distanceToB = Math.abs(b - input);

    return distanceToA <= distanceToB;
  }

  public static double average(double... numbers) {
    return Arrays.stream(numbers).sum() / numbers.length;
  }
  // #endregion

  // #region LOGGING
  public static void logMotorStats(String key, SparkBase spark, boolean absoluteEncoder) {
    Logger.recordOutput(key + "/Position", Rotations.of(spark.getEncoder().getPosition()));
    Logger.recordOutput(key + "/Velocity", RPM.of(spark.getEncoder().getVelocity()));

    if (absoluteEncoder) {
      Logger.recordOutput(
          key + "/AbsPosition", Rotations.of(spark.getAbsoluteEncoder().getPosition()));
      Logger.recordOutput(key + "/AbsVelocity", RPM.of(spark.getAbsoluteEncoder().getVelocity()));
    }

    Logger.recordOutput(key + "/AppliedOutput", spark.getAppliedOutput());
    Logger.recordOutput(key + "/BusVoltage", Volts.of(spark.getBusVoltage()));
    Logger.recordOutput(
        key + "/AppliedVoltage", Volts.of(spark.getAppliedOutput() * spark.getBusVoltage()));
    Logger.recordOutput(key + "/Temp", Celsius.of(spark.getMotorTemperature()));
    Logger.recordOutput(key + "/Current", Amps.of(spark.getOutputCurrent()));

    Logger.recordOutput(key + "/Setpoint", spark.getClosedLoopController().getSetpoint());
    Logger.recordOutput(key + "/AtSetpoint", spark.getClosedLoopController().isAtSetpoint());

    logFaults(key + "/Faults", spark.getFaults());
    logWarnings(key + "/Warnings", spark.getWarnings());

    if (tuningMode) {
      logFaults(key + "/StickyFaults", spark.getStickyFaults());
      logWarnings(key + "/StickyWarnings", spark.getStickyWarnings());
    }
  }

  public static void logMotorStatsNotCoveredByIO(String key, SparkBase spark) {
    Logger.recordOutput(key + "/AppliedOutput", spark.getAppliedOutput());
    Logger.recordOutput(key + "/BusVoltage", Volts.of(spark.getBusVoltage()));
    Logger.recordOutput(key + "/Temp", Celsius.of(spark.getMotorTemperature()));

    Logger.recordOutput(key + "/Setpoint", spark.getClosedLoopController().getSetpoint());
    Logger.recordOutput(key + "/AtSetpoint", spark.getClosedLoopController().isAtSetpoint());

    logFaults(key + "/Faults", spark.getFaults());
    logWarnings(key + "/Warnings", spark.getWarnings());

    if (tuningMode) {
      logFaults(key + "/StickyFaults", spark.getStickyFaults());
      logWarnings(key + "/StickyWarnings", spark.getStickyWarnings());
    }
  }

  private static void logFaults(String key, Faults faults) {
    Logger.recordOutput(key + "/Count", Integer.bitCount(faults.rawBits));

    Logger.recordOutput(key + "/Can", faults.can);
    Logger.recordOutput(key + "/EscEeprom", faults.escEeprom);
    Logger.recordOutput(key + "/Firmware", faults.firmware);
    Logger.recordOutput(key + "/GateDriver", faults.gateDriver);
    Logger.recordOutput(key + "/MotorType", faults.motorType);
    Logger.recordOutput(key + "/Other", faults.other);
    Logger.recordOutput(key + "/Sensor", faults.sensor);
    Logger.recordOutput(key + "/Temperature", faults.temperature);
  }

  private static void logWarnings(String key, Warnings warnings) {
    Logger.recordOutput(key + "/Count", Integer.bitCount(warnings.rawBits));

    Logger.recordOutput(key + "/Brownout", warnings.brownout);
    Logger.recordOutput(key + "/EscEeprom", warnings.escEeprom);
    Logger.recordOutput(key + "/ExtEeprom", warnings.extEeprom);
    Logger.recordOutput(key + "/HasReset", warnings.hasReset);
    Logger.recordOutput(key + "/Other", warnings.other);
    Logger.recordOutput(key + "/Overcurrent", warnings.overcurrent);
    Logger.recordOutput(key + "/Sensor", warnings.sensor);
    Logger.recordOutput(key + "/Stall", warnings.stall);
  }
  // #endregion
}
