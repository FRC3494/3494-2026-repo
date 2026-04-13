package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;

public final class QuadranglesUtil {
  public static Pose2d flipPose(Pose2d pose) {
    return new Pose2d(flipTranslation(pose.getTranslation()), flipAngle(pose.getRotation()));
  }

  public static Pose2d toAlliancePose(Pose2d bluePose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return bluePose;
    } else {
      return flipPose(bluePose);
    }
  }

  public static Translation2d flipTranslation(Translation2d translation) {
    return fieldSize.minus(translation);
  }

  public static Translation2d toAllianceTranslation(Translation2d blueTranslation) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return blueTranslation;
    } else {
      return flipTranslation(blueTranslation);
    }
  }

  public static Rotation2d flipAngle(Rotation2d angle) {
    return angle.rotateBy(Rotation2d.k180deg);
  }

  public static Rotation2d toAllianceAngle(Rotation2d blueAngle) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return blueAngle;
    } else {
      return flipAngle(blueAngle);
    }
  }

  public static Distance toAllianceX(Distance blueX) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return blueX;
    } else {
      return fieldLength.minus(blueX);
    }
  }

  public static double toAllianceXMeters(double blueXMeters) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return blueXMeters;
    } else {
      return fieldLength.in(Meters) - blueXMeters;
    }
  }

  public static Distance toAllianceY(Distance blueY) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return blueY;
    } else {
      return fieldWidth.minus(blueY);
    }
  }

  public static double toAllianceYMeters(double blueYMeters) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
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

    Logger.recordOutput(key + "/Faults/Can", spark.getFaults().can);
    Logger.recordOutput(key + "/Faults/EscEeprom", spark.getFaults().escEeprom);
    Logger.recordOutput(key + "/Faults/Firmware", spark.getFaults().firmware);
    Logger.recordOutput(key + "/Faults/GateDriver", spark.getFaults().gateDriver);
    Logger.recordOutput(key + "/Faults/MotorType", spark.getFaults().motorType);
    Logger.recordOutput(key + "/Faults/Other", spark.getFaults().other);
    Logger.recordOutput(key + "/Faults/Sensor", spark.getFaults().sensor);
    Logger.recordOutput(key + "/Faults/Temperature", spark.getFaults().temperature);

    Logger.recordOutput(key + "/Warnings/Brownout", spark.getWarnings().brownout);
    Logger.recordOutput(key + "/Warnings/EscEeprom", spark.getWarnings().escEeprom);
    Logger.recordOutput(key + "/Warnings/ExtEeprom", spark.getWarnings().extEeprom);
    Logger.recordOutput(key + "/Warnings/HasReset", spark.getWarnings().hasReset);
    Logger.recordOutput(key + "/Warnings/Other", spark.getWarnings().other);
    Logger.recordOutput(key + "/Warnings/Overcurrent", spark.getWarnings().overcurrent);
    Logger.recordOutput(key + "/Warnings/Sensor", spark.getWarnings().sensor);
    Logger.recordOutput(key + "/Warnings/Stall", spark.getWarnings().stall);
  }
}
