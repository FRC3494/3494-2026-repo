package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;
import static frc.robot.util.QuadranglesUtil.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.util.choreo.ChoreoVars;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Autos {
  // #region LOGGING
  public static void logTrajectory(Trajectory<SwerveSample> trajectory, boolean starting) {
    Logger.recordOutput("Choreo/TrajStarting", starting);
    Logger.recordOutput("Choreo/TrajName", trajectory.name());
    Logger.recordOutput(
        "Choreo/TrajInitPose",
        trajectory.getInitialPose(mirrorForRedAlliance).orElse(Pose2d.kZero));
    Logger.recordOutput(
        "Choreo/TrajFinalPose", trajectory.getFinalPose(mirrorForRedAlliance).orElse(Pose2d.kZero));
    Logger.recordOutput("Choreo/TrajTotalTime", trajectory.getTotalTime());

    Pose2d[] originalTrajPoses = trajectory.getPoses();
    Pose2d[] flippedTrajPoses = new Pose2d[originalTrajPoses.length];
    for (int i = 0; i < flippedTrajPoses.length; i++) {
      flippedTrajPoses[i] = toAlliancePose(originalTrajPoses[i]);
    }
    Logger.recordOutput("Choreo/TrajPoses", flippedTrajPoses);
  }
  // #endregion

  // #region COMMANDS
  public static Command climbDepot(
      RobotCommands robotCommands, Drive drive, ShooterAimModel shooterAimModel) {
    return sequence(
            AutoAlignCommand.alignSequence(
                drive,
                alliance == Alliance.Blue ? climbSetupPoseDepot_BLUE : climbSetupPoseDepot_RED,
                alliance == Alliance.Blue ? climbPoseDepot_BLUE : climbPoseDepot_RED),
            parallel(
                robotCommands.creepBackward(),
                sequence(
                    waitUntil(() -> Timer.getMatchTime() <= climbTime)
                        .deadlineFor(robotCommands.shoot()),
                    runOnce(
                        () -> {
                          shooterAimModel.setTurretTrim(
                              turretTrimDefaultRot + Units.degreesToRotations(-5.0));
                        },
                        shooterAimModel),
                    parallel(robotCommands.runClimberMidWithCurrent(), robotCommands.runIntakeUp()),
                    runOnce(
                        () -> {
                          shooterAimModel.setTurretTrim(
                              turretTrimDefaultRot + Units.degreesToRotations(-10.0));
                        },
                        shooterAimModel),
                    robotCommands.shoot())))
        .finallyDo(
            () -> {
              shooterAimModel.setTurretTrim(turretTrimDefaultRot);
            });
  }

  public static Command climbOutpost(
      RobotCommands robotCommands, Drive drive, ShooterAimModel shooterAimModel) {
    return sequence(
            AutoAlignCommand.alignSequence(
                drive,
                alliance == Alliance.Blue ? climbSetupPoseOutpost_BLUE : climbSetupPoseOutpost_RED,
                alliance == Alliance.Blue ? climbPoseOutpost_BLUE : climbPoseOutpost_RED),
            parallel(
                robotCommands.creepBackward(),
                sequence(
                    waitUntil(() -> Timer.getMatchTime() <= climbTime)
                        .deadlineFor(robotCommands.shoot()),
                    runOnce(
                        () -> {
                          shooterAimModel.setTurretTrim(
                              turretTrimDefaultRot + Units.degreesToRotations(5.0));
                        },
                        shooterAimModel),
                    parallel(robotCommands.runClimberMidWithCurrent(), robotCommands.runIntakeUp()),
                    runOnce(
                        () -> {
                          shooterAimModel.setTurretTrim(
                              turretTrimDefaultRot + Units.degreesToRotations(10.0));
                        },
                        shooterAimModel),
                    robotCommands.shoot())))
        .finallyDo(
            () -> {
              shooterAimModel.setTurretTrim(turretTrimDefaultRot);
            });
  }
  // #endregion

  private static Command resetOdoForAuto(Drive drive, Pose2d pose) {
    return runOnce(
            () -> {
              drive.setPose(toAlliancePose(pose));
            },
            drive)
        .ignoringDisable(true)
        .withName("ResetOdoForAuto");
  }

  public static Command resetOdoLeftTrench(Drive drive) {
    return resetOdoForAuto(drive, ChoreoVars.Poses.LeftTrenchStartingPosition)
        .withName("ResetOdoLeftTrench");
  }

  public static Command resetOdoRightTrench(Drive drive) {
    return resetOdoForAuto(drive, ChoreoVars.Poses.RightTrenchStartingPosition)
        .withName("ResetOdoRightTrench");
  }

  public static Command resetOdoLeftBump(Drive drive) {
    return resetOdoForAuto(drive, ChoreoVars.Poses.LeftBumpStartingPosition)
        .withName("ResetOdoLeftBump");
  }

  public static Command resetOdoRightBump(Drive drive) {
    return resetOdoForAuto(drive, ChoreoVars.Poses.RightBumpStartingPosition)
        .withName("ResetOdoRightBump");
  }

  // #region LOADING
  public static void loadAuto(
      AutoBase auto,
      HashMap<String, Pose2d> startingPoseMap,
      AutoChooser autoChooser,
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {
    autoChooser.addRoutine(
        auto.getName() + "_BLUE",
        () ->
            auto.getRoutine(
                auto.getName() + "_BLUE",
                Alliance.Blue,
                autoFactory,
                robotCommands,
                drive,
                shooterAimModel));
    autoChooser.addRoutine(
        auto.getName() + "_RED",
        () ->
            auto.getRoutine(
                auto.getName() + "_RED",
                Alliance.Red,
                autoFactory,
                robotCommands,
                drive,
                shooterAimModel));

    startingPoseMap.put(auto.getName() + "_BLUE", auto.getStartingPose());
    startingPoseMap.put(auto.getName() + "_RED", auto.getStartingPose());
  }
  // #endregion
}
