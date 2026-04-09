package frc.robot.subsystems.drive.autoalign;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.QuadranglesUtil;
import java.util.Set;

public class AutoAlignToTargetCommands {
  public static Command autoAlignToTower(Drive drive, RobotCommands robotCommands) {
    return either(
            sequence(
                AutoAlignCommand.alignSequenceDeferred(
                    drive, climbSetupPoseOutpost, climbPoseOutpost),
                robotCommands.creepBackward()),
            sequence(
                AutoAlignCommand.alignSequenceDeferred(drive, climbSetupPoseDepot, climbPoseDepot),
                robotCommands.creepBackward()),
            () ->
                QuadranglesUtil.closerToWithFlip(
                    climbSetupPoseOutpost.getTranslation(),
                    climbSetupPoseDepot.getTranslation(),
                    drive.getPose().getTranslation()))
        .withName("AutoAlignTower");
  }

  public static Command autoDriveThroughTrench(Drive drive, RobotCommands robotCommands) {
    return sequence(
            robotCommands.stopShootNoDelay(),
            either(
                autoDriveTrench(drive, robotCommands, closeLeftTrench, closeRightTrench),
                autoDriveTrench(drive, robotCommands, farLeftTrench, farRightTrench),
                () ->
                    QuadranglesUtil.toAllianceX(drive.getPose().getMeasureX())
                        .lt(closerToOppositeTrenchLine)))
        .withName("AutoDriveThruTrench");
  }

  private static Command autoDriveTrench(
      Drive drive,
      RobotCommands robotCommands,
      Translation2d leftTrench,
      Translation2d rightTrench) {
    return either(
        autoDriveTrench(drive, robotCommands, leftTrench),
        autoDriveTrench(drive, robotCommands, rightTrench),
        () ->
            QuadranglesUtil.closerToWithFlip(
                leftTrench, rightTrench, drive.getPose().getTranslation()));
  }

  private static Command autoDriveTrench(
      Drive drive, RobotCommands robotCommands, Translation2d trench) {
    return either(
        sequence(
            defer(
                () ->
                    AutoAlignCommand.alignSequence(
                        drive,
                        trenchLinearTolerance,
                        trenchAngularTolerance,
                        new Pose2d(
                            trench.minus(new Translation2d(preTrenchOffset, Meters.zero())),
                            closestTrenchOrientation(drive.getRotation())),
                        new Pose2d(
                            trench.plus(new Translation2d(postTrenchOffset, Meters.zero())),
                            closestTrenchOrientation(drive.getRotation()))),
                Set.of(drive)),
            robotCommands.stopDrive(),
            run(() -> {})),
        sequence(
            defer(
                () ->
                    AutoAlignCommand.alignSequence(
                        drive,
                        trenchLinearTolerance,
                        trenchAngularTolerance,
                        new Pose2d(
                            trench.plus(new Translation2d(preTrenchOffset, Meters.zero())),
                            closestTrenchOrientation(drive.getRotation())),
                        new Pose2d(
                            trench.minus(new Translation2d(postTrenchOffset, Meters.zero())),
                            closestTrenchOrientation(drive.getRotation()))),
                Set.of(drive)),
            robotCommands.stopDrive(),
            run(() -> {})),
        () -> QuadranglesUtil.toAllianceX(drive.getPose().getMeasureX()).lt(trench.getMeasureX()));
  }

  private static Rotation2d closestTrenchOrientation(Rotation2d robotYaw) {
    if (Math.abs(MathUtil.angleModulus(robotYaw.getRadians())) < Math.PI / 2.0) {
      return QuadranglesUtil.toAllianceAngle(Rotation2d.kZero);
    } else {
      return QuadranglesUtil.toAllianceAngle(Rotation2d.k180deg);
    }
  }
}
