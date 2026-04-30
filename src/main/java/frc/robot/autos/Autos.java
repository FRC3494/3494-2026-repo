package frc.robot.autos;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.DriveConstants.AutoAlignConstants.*;
import static frc.robot.Constants.ShooterConstants.AimShooterMathLinearConstants.*;
import static frc.robot.util.QuadranglesUtil.*;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.autoalign.AutoAlignCommand;
import frc.robot.subsystems.shooter.ShooterAimModel;
import frc.robot.util.choreo.ChoreoVars;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class Autos {
  public record AutoRequirements(
      AutoFactory autoFactory,
      RobotCommands robotCommands,
      Drive drive,
      ShooterAimModel shooterAimModel) {}

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
  public static Command climbDepot(AutoRequirements requirements) {
    return climbCommand(
            requirements,
            climbSetupPoseDepot_BLUE,
            climbSetupPoseDepot_RED,
            climbPoseDepot_BLUE,
            climbPoseDepot_RED)
        .withName("ClimbDepot");
  }

  public static Command climbOutpost(AutoRequirements requirements) {
    return climbCommand(
            requirements,
            climbSetupPoseOutpost_BLUE,
            climbSetupPoseOutpost_RED,
            climbPoseOutpost_BLUE,
            climbPoseOutpost_RED)
        .withName("ClimbOutpost");
  }

  private static Command climbCommand(
      AutoRequirements requirements,
      Pose2d setupPose_BLUE,
      Pose2d setupPose_RED,
      Pose2d climbPose_BLUE,
      Pose2d climbPose_RED) {
    return sequence(
            sequence(
                    defer(
                        () ->
                            new AutoAlignCommand(
                                alliance == Alliance.Blue ? setupPose_BLUE : setupPose_RED,
                                requirements.drive(),
                                autoAlignLinearTolerance,
                                Meters.of(5),
                                autoAlignAngularTolerance),
                        Set.of(requirements.drive())),
                    defer(
                        () ->
                            new AutoAlignCommand(
                                alliance == Alliance.Blue ? climbPose_BLUE : climbPose_RED,
                                requirements.drive()),
                        Set.of(requirements.drive())))
                .deadlineFor(requirements.robotCommands().shoot()),
            parallel(
                requirements.robotCommands().creepBackward(),
                sequence(
                    waitUntil(() -> Timer.getMatchTime() <= climbTime)
                        .deadlineFor(requirements.robotCommands().shoot()),
                    parallel(
                        requirements.robotCommands().runClimberMidWithCurrent(),
                        requirements.robotCommands().runIntakeUp()),
                    requirements.robotCommands().shoot())))
        .finallyDo(
            () -> {
              requirements.shooterAimModel().setTurretTrim(turretTrimDefaultRot);
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
}
