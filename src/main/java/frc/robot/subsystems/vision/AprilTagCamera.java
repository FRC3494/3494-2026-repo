package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.maxTagDistance;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AprilTagCamera {
  private String name;

  private double heartbeat;

  @Getter @Setter @AutoLogOutput private boolean megaTag2Enabled;

  @Setter
  @AutoLogOutput(key = "Vision/{name}/RobotYaw")
  private Rotation2d robotYaw = Rotation2d.kZero;

  @Setter
  @AutoLogOutput(key = "Vision/{name}/RobotYawVelocity")
  private AngularVelocity robotYawVelocity = DegreesPerSecond.of(0.0);

  @Getter private Pose2d pose;
  @Getter private double poseTimestamp;
  @Getter private Matrix<N3, N1> stdDevs;

  public AprilTagCamera(String name, boolean megaTag2) {
    this.name = name;
    this.megaTag2Enabled = megaTag2;
  }

  public void periodic() {
    // Determine whether camera is connected
    double newHeartbeat = LimelightHelpers.getHeartbeat(name);
    Logger.recordOutput(name + "/Connected", newHeartbeat != heartbeat);
    heartbeat = newHeartbeat;

    LimelightHelpers.SetRobotOrientation(
        name, robotYaw.getDegrees(), robotYawVelocity.in(DegreesPerSecond), 0, 0, 0, 0);

    PoseEstimate poseEstimate;
    if (megaTag2Enabled) {
      poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    } else {
      poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    Logger.recordOutput("Vision/" + name + "/AvgTagDist", poseEstimate.avgTagDist);
    Logger.recordOutput("Vision/" + name + "/AvgTagArea", poseEstimate.avgTagArea);
    Logger.recordOutput("Vision/" + name + "/Pose", poseEstimate.pose);
    Logger.recordOutput("Vision/" + name + "/TagCount", poseEstimate.tagCount);
    Logger.recordOutput("Vision/" + name + "/IsMegaTag2", poseEstimate.isMegaTag2);
    Logger.recordOutput("Vision/" + name + "/Latency", poseEstimate.latency);
    Logger.recordOutput("Vision/" + name + "/TagCount", poseEstimate.tagCount);
    Logger.recordOutput("Vision/" + name + "/TagSpan", poseEstimate.tagSpan);
    Logger.recordOutput("Vision/" + name + "/PoseTimestamp", poseEstimate.timestampSeconds);

    Logger.recordOutput("Vision/" + name + "/RawFiducials", poseEstimate.rawFiducials.toString());

    boolean estimateNotNull = poseEstimate != null;
    boolean tagsPresent = poseEstimate.tagCount > 0;
    boolean tagsWithinRange = poseEstimate.avgTagDist < maxTagDistance.in(Meters);
    boolean validMeasurement = estimateNotNull && tagsPresent && tagsWithinRange;

    Logger.recordOutput("Vision/" + name + "/EstimateNotNull", estimateNotNull);
    Logger.recordOutput("Vision/" + name + "/TagsPresent", tagsPresent);
    Logger.recordOutput("Vision/" + name + "/TagsWithinRange", tagsWithinRange);
    Logger.recordOutput("Vision/" + name + "/ValidMeasurement", validMeasurement);

    if (validMeasurement) {
      stdDevs =
          // VecBuilder.fill(
          //     maxDistanceStdDev * (poseEstimate.avgTagDist / maxTagDistance.in(Meters)),
          //     maxDistanceStdDev * (poseEstimate.avgTagDist / maxTagDistance.in(Meters)),
          //     maxAngleStdDev.in(Radians)
          //         * Math.abs(
          //             poseEstimate.pose.getRotation().minus(drive.getRotation()).getRadians()
          //                 / (2 * Math.PI)));
          VecBuilder.fill(.1, .1, 9999999);
      Logger.recordOutput("Vision/" + name + "/StdDevs", stdDevs);

      pose = poseEstimate.pose;
      poseTimestamp = poseEstimate.timestampSeconds;
    }
  }
}
