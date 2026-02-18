package frc.robot.autos;

import static frc.robot.Constants.DriveConstants.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.QuadranglesUtil;
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
      flippedTrajPoses[i] = QuadranglesUtil.toAlliancePose(originalTrajPoses[i]);
    }
    Logger.recordOutput("Choreo/TrajPoses", flippedTrajPoses);
  }
}
