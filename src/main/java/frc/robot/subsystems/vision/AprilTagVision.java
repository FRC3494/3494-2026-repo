package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.drive.Drive;

public class AprilTagVision extends SubsystemBase {
  private final AprilTagCamera[] cameras = new AprilTagCamera[RobotMap.aprilTagLimelights.length];
  private final Drive drive;
  private final boolean megaTagTwo;

  public AprilTagVision(Drive drive) {
    this.drive = drive;

    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new AprilTagCamera(RobotMap.aprilTagLimelights[i], true);
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
    }
  }
}
