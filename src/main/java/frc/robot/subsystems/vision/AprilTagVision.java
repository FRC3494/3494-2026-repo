package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class AprilTagVision extends SubsystemBase {
  private String[] names;

  private Drive drive;

  public AprilTagVision(String[] names, Drive drive) {
    this.names = names;
    this.drive = drive;
  }

  @Override
  public void periodic() {
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
      if (true) {
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      } else {
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      }

      boolean validMeasurement =
          poseEstimate != null
              && poseEstimate.tagCount > 0
              && poseEstimate.avgTagDist < VisionConstants.maxTagDistance.in(Meters);

      if (validMeasurement) {
        drive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
      }
    }
  }
}
