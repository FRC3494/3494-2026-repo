package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayEntry;
import frc.robot.util.LimelightHelpers.LimelightResults;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

// import com.revrobotics.REVLibError;
// import com.revrobotics.spark.SparkBase;
// import java.util.function.Consumer;
// import java.util.function.DoubleConsumer;
// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.Logger;

public class LogUtil {
  public static void logMotorStats(String key, SparkBase spark, boolean absoluteEncoder) {
    if (absoluteEncoder) {
      Logger.recordOutput(
          key + "/Position", Rotations.of(spark.getAbsoluteEncoder().getPosition()));
      Logger.recordOutput(key + "/Velocity", RPM.of(spark.getAbsoluteEncoder().getVelocity()));
    } else {
      Logger.recordOutput(key + "/Position", Rotations.of(spark.getEncoder().getPosition()));
      Logger.recordOutput(key + "/Velocity", RPM.of(spark.getEncoder().getVelocity()));
    }

    Logger.recordOutput(key + "/AppliedOutput", spark.getAppliedOutput());
    Logger.recordOutput(key + "/BusVoltage", Volts.of(spark.getBusVoltage()));
    Logger.recordOutput(key + "/Temp", Celsius.of(spark.getMotorTemperature()));
  }

  public static void logCameraStats(
      String key,
      String limelightName,
      PoseEstimate poseEstimate,
      boolean validMeasurement,
      boolean megaTagTwo,
      Matrix<N3, N1> measurementStdDevs) {
    Logger.recordOutput(key + "/Latency", poseEstimate.latency);
    Logger.recordOutput(key + "/TagCount", poseEstimate.tagCount);
    Logger.recordOutput(key + "/AvgTagDist", Meters.of(poseEstimate.avgTagDist));
    Logger.recordOutput(key + "/validMeasurement", validMeasurement);
    Logger.recordOutput(key + "/megaTagTwo", megaTagTwo);
    Logger.recordOutput(key + "/StdDevX", Meters.of(measurementStdDevs.get(0, 0)));
    Logger.recordOutput(key + "/StdDevY", Meters.of(measurementStdDevs.get(1, 0)));
    Logger.recordOutput(key + "/StdDevTheta", Radians.of(measurementStdDevs.get(2, 0)));

    LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
    if (results != null && results.hardware != null) {
      Logger.recordOutput(key + "/CPUTemp", Celsius.of(results.hardware.temperature));
      Logger.recordOutput(key + "/CPUUsage", Percent.of(results.hardware.cpuUsage));
      Logger.recordOutput(key + "/RAMUsage", Percent.of(results.hardware.ramUsage));
      Logger.recordOutput(key + "/DiskFree", results.hardware.diskFree);
    }
  }
}
