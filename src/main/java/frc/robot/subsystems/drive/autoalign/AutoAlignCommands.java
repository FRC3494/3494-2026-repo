package frc.robot.subsystems.drive.autoalign;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
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
        () ->
            QuadranglesUtil.closerToWithFlip(
                climbSetupPoseOutpost.getTranslation(),
                climbSetupPoseDepot.getTranslation(),
                drive.getPose().getTranslation()));
  }
}
