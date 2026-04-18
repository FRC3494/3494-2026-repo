package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  private AprilTagCamera[] cameras =
      new AprilTagCamera[RobotMap.VisionConstants.aprilTagLimelights.length];

  private Drive drive;

  public AprilTagVision(Drive drive) {
    this.drive = drive;

    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new AprilTagCamera(RobotMap.VisionConstants.aprilTagLimelights[i], useMegaTag2);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.tuningMode) {
      builder.addDoubleProperty(
          "Distance Std Dev", () -> distanceStdDev, (double value) -> distanceStdDev = value);
      builder.addDoubleProperty(
          "Angle Std Dev", () -> angleStdDev, (double value) -> angleStdDev = value);
    }

    Logger.recordOutput("Vision/DistanceStdDev", distanceStdDev);
    Logger.recordOutput("Vision/AngleStdDev", angleStdDev);
  }

  @Override
  public void periodic() {
    for (AprilTagCamera camera : cameras) {
      if (useMegaTag2 && drive.isGyroConnected() != camera.isMegaTag2Enabled()) {
        camera.setMegaTag2Enabled(drive.isGyroConnected());
      }

      camera.setRobotYaw(drive.getRotation());
      camera.setRobotYawVelocity(RadiansPerSecond.of(drive.getYawVelocityRadPerSec()));

      camera.update();

      int limelightsBeingUsed = 0;
      if (camera.isValidMeasurement()) {
        limelightsBeingUsed += 1;
        drive.addVisionMeasurement(
            camera.getPose(),
            camera.getPoseTimestamp(),
            VecBuilder.fill(distanceStdDev, distanceStdDev, angleStdDev));
        Logger.recordOutput("Vision/" + camera.getName() + "/PoseInUse", camera.getPose());
      } else {
        Logger.recordOutput("Vision/" + camera.getName() + "/PoseInUse", Pose2d.kZero);
      }
      Logger.recordOutput("Vision/LimelightsBeingUsed", limelightsBeingUsed);
      Logger.recordOutput("Vision/SeeingTags", limelightsBeingUsed > 0);
    }
  }
}
