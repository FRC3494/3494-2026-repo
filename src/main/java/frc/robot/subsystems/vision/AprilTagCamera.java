package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.maxTagDistance;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.VisionConstants.LimelightConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AprilTagCamera {
  @Getter private final String name;

  @Getter private final Pose3d position;

  private double heartbeat;

  @Getter @Setter @AutoLogOutput private boolean megaTag2Enabled;

  @Setter
  @AutoLogOutput(key = "Vision/{name}/RobotYaw")
  private Rotation2d robotYaw = Rotation2d.kZero;

  @Setter
  @AutoLogOutput(key = "Vision/{name}/RobotYawVelocity")
  private AngularVelocity robotYawVelocity = DegreesPerSecond.of(0.0);

  @Getter
  @AutoLogOutput(key = "Vision/{name}/ValidMeasurement")
  private boolean validMeasurement1;
  @Getter
  @AutoLogOutput(key = "Vision/{name}/ValidMeasurement2")
  private boolean validMeasurement2;

  @Getter private Pose2d pose;
  @Getter private double poseTimestamp;
  @Getter private Matrix<N3, N1> stdDevs;

  public AprilTagCamera(LimelightConstants limelightConstants, boolean megaTag2) {
    this.name = limelightConstants.name();
    this.position = limelightConstants.position();

    this.megaTag2Enabled = megaTag2;

    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        position.getMeasureX().in(Meters),
        position.getMeasureY().in(Meters),
        position.getMeasureZ().in(Meters),
        position.getRotation().getMeasureX().in(Degrees),
        position.getRotation().getMeasureY().in(Degrees),
        position.getRotation().getMeasureZ().in(Degrees));
  }

  // ! This fn MUST be manually called since AprilTagCamera is not a SubsystemBase
  public void periodic() {
    // Determine whether Limelight is connected
    double newHeartbeat = LimelightHelpers.getHeartbeat(name);
    Logger.recordOutput("Vision/" + name + "/Connected", newHeartbeat != heartbeat);
    heartbeat = newHeartbeat;

    // Give robot yaw info to Limelight
    LimelightHelpers.SetRobotOrientation(
        name, robotYaw.getDegrees(), robotYawVelocity.in(DegreesPerSecond), 0, 0, 0, 0);

    // Get pose estimate from Limelight
    PoseEstimate poseEstimate2;
    PoseEstimate poseEstimate1;
    poseEstimate2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    poseEstimate1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);


    logPoseEstimateStats(poseEstimate2);
    logPoseEstimateStats(poseEstimate1);

    validMeasurement1 = isMeasurementValid(poseEstimate1);
    validMeasurement2 = isMeasurementValid(poseEstimate2);

    // Save pose estimate if valid
    if (validMeasurement1) {
      pose = poseEstimate1.pose;
      poseTimestamp = poseEstimate1.timestampSeconds;
      stdDevs = getStdDevs(poseEstimate1);
    }

    if (validMeasurement2) {
      pose = poseEstimate2.pose;
      poseTimestamp = poseEstimate2.timestampSeconds;
      stdDevs = getStdDevs(poseEstimate2);
    }
  }

  private void logPoseEstimateStats(PoseEstimate poseEstimate, String tagType) {
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/AvgTagDist", poseEstimate.avgTagDist);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/AvgTagArea", poseEstimate.avgTagArea);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/Pose", poseEstimate.pose);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/TagCount", poseEstimate.tagCount);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/IsMegaTag2", poseEstimate.isMegaTag2);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/Latency", poseEstimate.latency);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/TagCount", poseEstimate.tagCount);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/TagSpan", poseEstimate.tagSpan);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/PoseTimestamp", poseEstimate.timestampSeconds);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/RawFiducials", poseEstimate.rawFiducials.toString());
    Logger.recordOutput(
        "Vision/" + name + "/" + tagType + "/CameraPoseRobotSpace",
        LimelightHelpers.getCameraPose3d_RobotSpace(name));
  }

  private boolean isMeasurementValid(PoseEstimate poseEstimate) {
    // TODO: use limelight's own measurement valid function

    boolean estimateNotNull = poseEstimate != null;
    Logger.recordOutput("Vision/" + name + "/EstimateNotNull", estimateNotNull);

    boolean tagsPresent = poseEstimate.tagCount > 0;
    Logger.recordOutput("Vision/" + name + "/TagsPresent", tagsPresent);

    boolean tagsWithinRange = poseEstimate.avgTagDist < maxTagDistance.in(Meters);
    Logger.recordOutput("Vision/" + name + "/TagsWithinRange", tagsWithinRange);

    return estimateNotNull && tagsPresent && tagsWithinRange;
  }

  private Matrix<N3, N1> getStdDevs(PoseEstimate poseEstimate) {
    double standardDeviationX =
        .4; // maxDistanceStdDev * (poseEstimate.avgTagDist / maxTagDistance.in(Meters));
    double standardDeviationY =
        .4; // maxDistanceStdDev * (poseEstimate.avgTagDist / maxTagDistance.in(Meters));
    double standardDeviationTheta = 99999999; // maxAngleStdDev.in(Radians) *
    // Math.abs(poseEstimate.pose.getRotation().minus(drive.getRotation()).getRadians() / (2 *
    // Math.PI)));

    Logger.recordOutput("Vision/" + name + "/StdDevX", standardDeviationX);
    Logger.recordOutput("Vision/" + name + "/StdDevY", standardDeviationY);
    Logger.recordOutput("Vision/" + name + "/StdDevTheta", standardDeviationTheta);

    return VecBuilder.fill(standardDeviationX, standardDeviationY, standardDeviationTheta);
  }
}
