package frc.robot.subsystems.drive.autoalign;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI.DriveOI;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.util.QuadranglesUtil;

public class AutoAlignCommands {
  public static Command autoAlignToTower(Drive drive, RobotCommands robotCommands) {
    return either(
        deferredProxy(
            () ->
                sequence(
                    new AutoAlignCommand(climbSetupPoseOutpost, drive),
                    new AutoAlignCommand(climbPoseOutpost, drive),
                    robotCommands.creepBackward())),
        deferredProxy(
            () ->
                sequence(
                    new AutoAlignCommand(climbSetupPoseDepot, drive),
                    new AutoAlignCommand(climbPoseDepot, drive),
                    robotCommands.creepBackward())),
        () -> closerToOutpostClimb(drive.getPose().getTranslation()));
  }

  // TODO: make generic closerTo and closerToBlueTranslations in QuadranglesUtil
  private static boolean closerToOutpostClimb(Translation2d robotLocation) {
    double distanceToOutpostPose =
        robotLocation.getDistance(
            QuadranglesUtil.toAllianceTranslation(climbSetupPoseOutpost.getTranslation()));
    double distanceToDepotPose =
        robotLocation.getDistance(
            QuadranglesUtil.toAllianceTranslation(climbSetupPoseDepot.getTranslation()));

    return distanceToOutpostPose < distanceToDepotPose;
  }

  public static Command driveThroughTrench(Drive drive) {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        DriveOI::joystickDriveX,
        DriveOI::joystickDriveY,
        () -> closestTrenchOrientation(drive.getRotation()));
  }

  private static Rotation2d closestTrenchOrientation(Rotation2d robotYaw) {
    if (Math.abs(MathUtil.angleModulus(robotYaw.getRadians())) < Math.PI / 4.0) {
      return Rotation2d.kZero;
    } else {
      return Rotation2d.k180deg;
    }
  }
}
