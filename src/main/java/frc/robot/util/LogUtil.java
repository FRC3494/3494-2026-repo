package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase;
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

    public static void logCameraStats(String key, String camera) {
      Logger.recordOutput(key + "/IsConnected", camera.isConnected());
      Logger.recordOutput(key + "/Latency", Milliseconds.of(camera.getLatency()));
      Logger.recordOutput(key + "/TargetArea", camera.getTargetArea());
      Logger.recordOutput(key + "/TargetXOffset", Degrees.of(camera.getTargetXOffset()));
      Logger.recordOutput(key + "/TargetYOffset", Degrees.of(camera.getTargetYOffset())); 
  }
}
