package frc.robot.util;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class QuadranglesUtil {
  public static Pose2d toAlliancePose(Pose2d bluePose) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      return new Pose2d(
          toAllianceTranslation(bluePose.getTranslation()),
          bluePose.getRotation().rotateBy(Rotation2d.k180deg));
    } else {
      return bluePose;
    }
  }

  public static Translation2d toAllianceTranslation(Translation2d blueTranslation) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      return fieldSize.minus(blueTranslation);
    } else {
      return blueTranslation;
    }
  }
}
