package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  @Getter
  @Setter
  @AutoLogOutput(key = "Vision/{name}/MegaTag2Enabled")
  private boolean megaTag2Enabled;

  @Setter
  @AutoLogOutput(key = "Vision/{name}/RobotYaw")
  private Rotation2d robotYaw = Rotation2d.kZero;

  @Setter
  @AutoLogOutput(key = "Vision/{name}/RobotYawVelocity")
  private AngularVelocity robotYawVelocity = DegreesPerSecond.of(0.0);

  private boolean validMeasurement1 = false;
  private boolean validMeasurement2 = false;

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
    Logger.recordOutput(
        "Vision/" + name + "/CameraPoseRobotSpace",
        LimelightHelpers.getCameraPose3d_RobotSpace(name));

    LimelightHelpers.SetFiducialIDFiltersOverride(name, enabledAprilTags);
    Logger.recordOutput("Vision" + name + "/EnabledTags", enabledAprilTags);
  }

  // ! This fn MUST be manually called since AprilTagCamera is not a SubsystemBase
  public void update() {
    // Determine whether Limelight is connected
    double newHeartbeat = LimelightHelpers.getHeartbeat(name);
    Logger.recordOutput(
        "Vision/" + name + "/Connected", connectedDebouncer.calculate(newHeartbeat != heartbeat));
    heartbeat = newHeartbeat;

    if (megaTag2Enabled) {
      // Give robot yaw info to Limelight
      LimelightHelpers.SetRobotOrientation(
          name, robotYaw.getDegrees(), robotYawVelocity.in(DegreesPerSecond), 0, 0, 0, 0);

      PoseEstimate poseEstimate2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

      logPoseEstimateStats(poseEstimate2, true);

      validMeasurement2 = isMeasurementValid(poseEstimate2, true);
      Logger.recordOutput("Vision/" + name + "/ValidMeasurement2", validMeasurement2);

      if (validMeasurement2) {
        pose = poseEstimate2.pose;
        poseTimestamp = poseEstimate2.timestampSeconds;
        stdDevs = getStdDevs(poseEstimate2);
      }
    } else {
      PoseEstimate poseEstimate1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

      logPoseEstimateStats(poseEstimate1, false);

      validMeasurement1 = isMeasurementValid(poseEstimate1, false);
      Logger.recordOutput("Vision/" + name + "/ValidMeasurement1", validMeasurement1);

      if (validMeasurement1) {
        pose = poseEstimate1.pose;
        poseTimestamp = poseEstimate1.timestampSeconds;
        stdDevs = getStdDevs(poseEstimate1);
      }
    }
  }

  @AutoLogOutput(key = "Vision/{name}/Valid")
  public boolean isValidMeasurement() {
    if (megaTag2Enabled) {
      return validMeasurement2;
    } else {
      return validMeasurement1;
    }
  }

  private void logPoseEstimateStats(PoseEstimate poseEstimate, boolean megaTag2) {
    String tagType = megaTag2 ? "MegaTag2" : "MegaTag1";

    Logger.recordOutput("Vision/" + name + "/" + tagType + "/AvgTagDist", poseEstimate.avgTagDist);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/AvgTagArea", poseEstimate.avgTagArea);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/Pose", poseEstimate.pose);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/TagCount", poseEstimate.tagCount);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/IsMegaTag2", poseEstimate.isMegaTag2);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/Latency", poseEstimate.latency);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/TagCount", poseEstimate.tagCount);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/TagSpan", poseEstimate.tagSpan);
    Logger.recordOutput(
        "Vision/" + name + "/" + tagType + "/PoseTimestamp", poseEstimate.timestampSeconds);
    // Logger.recordOutput(
    //     "Vision/" + name + "/" + tagType + "/RawFiducials",
    // poseEstimate.rawFiducials.toString());
  }

  private boolean isMeasurementValid(PoseEstimate poseEstimate, boolean megaTag2) {
    // TODO: use limelight's own measurement valid function
    String tagType = megaTag2 ? "MegaTag2" : "MegaTag1";

    boolean estimateNotNull = poseEstimate != null;
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/EstimateNotNull", estimateNotNull);

    boolean tagCountValid =
        megaTag2
            ? poseEstimate.tagCount >= minTagCountMT2
            : poseEstimate.tagCount >= minTagCountMT1;
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/TagCountValid", tagCountValid);

    boolean tagsWithinRange =
        poseEstimate.avgTagDist < (megaTag2 ? maxTagDistanceMT2 : maxTagDistanceMT1).in(Meters);
    Logger.recordOutput("Vision/" + name + "/" + tagType + "/TagsWithinRange", tagsWithinRange);

    return estimateNotNull && tagCountValid && tagsWithinRange;
  }

  private Matrix<N3, N1> getStdDevs(PoseEstimate poseEstimate) {
    // TODO: different stdDevs for mt1/mt2

    double standardDeviationX = distanceStdDev; // maxDistanceStdDev * (poseEstimate.avgTagDist /
    // maxTagDistance.in(Meters));
    double standardDeviationY = distanceStdDev; // maxDistanceStdDev * (poseEstimate.avgTagDist /
    // maxTagDistance.in(Meters));
    double standardDeviationTheta = angleStdDev; // maxAngleStdDev *
    // Math.abs(poseEstimate.pose.getRotation().minus(drive.getRotation()).getRadians() / (2 *
    // Math.PI)));

    Logger.recordOutput("Vision/" + name + "/StdDevX", standardDeviationX);
    Logger.recordOutput("Vision/" + name + "/StdDevY", standardDeviationY);
    Logger.recordOutput("Vision/" + name + "/StdDevTheta", standardDeviationTheta);

    return VecBuilder.fill(standardDeviationX, standardDeviationY, standardDeviationTheta);
  }
}
