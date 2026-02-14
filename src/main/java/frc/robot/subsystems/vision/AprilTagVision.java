package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.drive.Drive;

public class AprilTagVision extends SubsystemBase {
  private AprilTagCamera[] cameras = new AprilTagCamera[RobotMap.aprilTagLimelights.length];

  private Drive drive;

  public AprilTagVision(Drive drive) {
    this.drive = drive;

    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new AprilTagCamera(RobotMap.aprilTagLimelights[i], useMegaTag2);
    }
  }

  @Override
  public void periodic() {
    for (AprilTagCamera camera : cameras) {
      if (useMegaTag2 && drive.isGyroConnected() != camera.isMegaTag2Enabled()) {
        camera.setMegaTag2Enabled(drive.isGyroConnected());
      }

      camera.setRobotYaw(drive.getRotation());
      camera.setRobotYawVelocity(RadiansPerSecond.of(drive.getYawVelocityRadPerSec()));

      camera.periodic();

      if (camera.isValidMeasurement()) {
        drive.addVisionMeasurement(
            camera.getPose(), camera.getPoseTimestamp(), camera.getStdDevs());
      }
    }
  }
}
