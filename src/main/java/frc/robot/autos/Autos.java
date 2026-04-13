package frc.robot.autos;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.util.QuadranglesUtil.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.choreo.ChoreoVars;
import org.littletonrobotics.junction.Logger;

public class Autos {
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
