package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class AprilTagVision extends SubsystemBase {
  private final AprilTagCamera[] cameras = new AprilTagCamera[RobotMap.aprilTagLimelightNames.length];
  private final Drive drive;
  private final boolean megaTagTwo;

  public AprilTagVision(Drive drive) {
    this.drive = drive;

    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new AprilTagCamera(RobotMap.aprilTagLimelightNames[i], true);
    }
    this.megaTagTwo = true;
  }

  @Override
  public void periodic() {
    for (AprilTagCamera camera : cameras) {
      camera.setRobotYaw(drive.getRotation());
      camera.setRobotYawVelocity(RadiansPerSecond.of(drive.getYawVelocityRadPerSec()));

      camera.periodic();

      drive.addVisionMeasurement(camera.getPose(), camera.getPoseTimestamp(), camera.getStdDevs());
      for (String name : names) {
        LimelightHelpers.SetRobotOrientation(
            name,
            drive.getRotation().getDegrees(),
            Units.radiansToDegrees(drive.getYawVelocityRadPerSec()),
            0,
            0,
            0,
            0);

        PoseEstimate poseEstimate;
        if (megaTagTwo) {
          poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        } else {
          poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        }

        boolean validMeasurement =
            poseEstimate != null
                && poseEstimate.tagCount > 0
                && poseEstimate.avgTagDist < maxTagDistance.in(Meters);

        if (validMeasurement) {
          Matrix<N3, N1> measurementStdDevs =
              // VecBuilder.fill(
              //     maxDistanceStdDev * (poseEstimate.avgTagDist / maxTagDistance.in(Meters)),
              //     maxDistanceStdDev * (poseEstimate.avgTagDist / maxTagDistance.in(Meters)),
              //     maxAngleStdDev.in(Radians)
              //         * Math.abs(
              //             poseEstimate.pose.getRotation().minus(drive.getRotation()).getRadians()
              //                 / (2 * Math.PI)));
              VecBuilder.fill(.1, .1, 9999999);

          // drive.addVisionMeasurement(
          //     poseEstimate.pose, poseEstimate.timestampSeconds, measurementStdDevs);

          frc.robot.util.LogUtil.logCameraStats(
              "Vision/", name, poseEstimate, validMeasurement, megaTagTwo, measurementStdDevs);
        }
      }
    }
  }
}
