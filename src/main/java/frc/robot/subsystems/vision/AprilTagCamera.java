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

  // ! This MUST be manually called since AprilTagCamera is not a SubsystemBase
  public void periodic() {
    // Determine whether Limelight is connected
    double newHeartbeat = LimelightHelpers.getHeartbeat(name);
    Logger.recordOutput(name + "/Connected", newHeartbeat != heartbeat);
    heartbeat = newHeartbeat;

    // Give robot yaw info to Limelight
    LimelightHelpers.SetRobotOrientation(
        name, robotYaw.getDegrees(), robotYawVelocity.in(DegreesPerSecond), 0, 0, 0, 0);

    // Get pose estimate from Limelight
    PoseEstimate poseEstimate;
    if (megaTag2Enabled) {
      poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    } else {
      poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    logPoseEstimateStats(poseEstimate);

    boolean validMeasurement = isMeasurementValid(poseEstimate);

    // Save pose estimate if valid
    if (validMeasurement) {
      pose = poseEstimate.pose;
      poseTimestamp = poseEstimate.timestampSeconds;
      stdDevs = getStdDevs(poseEstimate);
    }
  }

  private void logPoseEstimateStats(PoseEstimate poseEstimate) {
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
  }

  private boolean isMeasurementValid(PoseEstimate poseEstimate) {
    boolean estimateNotNull = poseEstimate != null;
    Logger.recordOutput("Vision/" + name + "/EstimateNotNull", estimateNotNull);

    boolean tagsPresent = poseEstimate.tagCount > 0;
    Logger.recordOutput("Vision/" + name + "/TagsPresent", tagsPresent);

    boolean tagsWithinRange = poseEstimate.avgTagDist < maxTagDistance.in(Meters);
    Logger.recordOutput("Vision/" + name + "/TagsWithinRange", tagsWithinRange);

    boolean validMeasurement = estimateNotNull && tagsPresent && tagsWithinRange;
    Logger.recordOutput("Vision/" + name + "/ValidMeasurement", validMeasurement);

    return validMeasurement;
  }

  private Matrix<N3, N1> getStdDevs(PoseEstimate poseEstimate) {
    Matrix<N3, N1> standardDeviations =
        // VecBuilder.fill(
        //     maxDistanceStdDev * (poseEstimate.avgTagDist / maxTagDistance.in(Meters)),
        //     maxDistanceStdDev * (poseEstimate.avgTagDist / maxTagDistance.in(Meters)),
        //     maxAngleStdDev.in(Radians)
        //         * Math.abs(
        //             poseEstimate.pose.getRotation().minus(drive.getRotation()).getRadians()
        //                 / (2 * Math.PI)));
        VecBuilder.fill(.1, .1, 9999999);

    Logger.recordOutput("Vision/" + name + "/StdDevs", standardDeviations);

    return standardDeviations;
  }
}
