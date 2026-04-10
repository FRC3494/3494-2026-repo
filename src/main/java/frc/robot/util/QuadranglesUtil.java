package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
}
